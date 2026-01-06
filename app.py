import streamlit as st
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter
import pandas as pd
import folium
from streamlit_folium import st_folium
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# --- CONFIGURAÇÃO DA PÁGINA ---
st.set_page_config(page_title="Otimizador de Rotas PRO", layout="wide")

# Inicializar memória
if 'locais' not in st.session_state:
    st.session_state['locais'] = []

# --- 1. FUNÇÕES MATEMÁTICAS ---

def buscar_coordenadas(endereco):
    """Converte texto em (lat, long)"""
    geolocator = Nominatim(user_agent="app_rotas_mvp_v2")
    try:
        # Limitamos a busca ao Brasil para melhorar precisão (pode remover se quiser mundial)
        location = geolocator.geocode(endereco, country_codes="BR")
        if location:
            return location.latitude, location.longitude
        return None
    except:
        return None

def calc_dist(lat1, lon1, lat2, lon2):
    """Haversine simples em Metros"""
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    # Fator 1.3 de tortuosidade (ruas não são retas)
    return int(2 * R * math.asin(math.sqrt(a)) * 1.3)

def calcular_rota_original_sequencial(lista):
    """Calcula a distância se o motorista seguisse a ordem da lista sem otimizar"""
    if len(lista) < 2: return 0
    dist_total = 0
    # De 0 para 1, 1 para 2, etc... e volta para 0
    for i in range(len(lista) - 1):
        dist_total += calc_dist(lista[i]['lat'], lista[i]['lon'], lista[i+1]['lat'], lista[i+1]['lon'])
    # Volta ao depósito
    dist_total += calc_dist(lista[-1]['lat'], lista[-1]['lon'], lista[0]['lat'], lista[0]['lon'])
    return dist_total

# --- 2. SOLVER (OR-TOOLS) ---
def resolver_otimizacao(lista_locais, num_veiculos):
    tamanho = len(lista_locais)
    matriz = [[0] * tamanho for _ in range(tamanho)]
    
    # Criar Matriz de Distâncias
    for i in range(tamanho):
        for j in range(tamanho):
            if i != j:
                matriz[i][j] = calc_dist(
                    lista_locais[i]['lat'], lista_locais[i]['lon'],
                    lista_locais[j]['lat'], lista_locais[j]['lon']
                )
    
    manager = pywrapcp.RoutingIndexManager(tamanho, num_veiculos, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return matriz[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Estratégia de Busca
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_parameters)
    
    resultados = []
    distancia_total_otimizada = 0

    if solution:
        for vehicle_id in range(num_veiculos):
            index = routing.Start(vehicle_id)
            rota_nomes = []
            rota_coords = []
            distancia_rota = 0
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                rota_nomes.append(lista_locais[node_index]['nome'])
                rota_coords.append([lista_locais[node_index]['lat'], lista_locais[node_index]['lon']])
                
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                distancia_rota += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            
            # Adicionar retorno ao depósito
            node_index = manager.IndexToNode(index)
            rota_nomes.append(lista_locais[node_index]['nome']) # Volta pro inicio
            rota_coords.append([lista_locais[node_index]['lat'], lista_locais[node_index]['lon']])
            
            resultados.append({
                "veiculo": vehicle_id + 1,
                "passos": rota_nomes,
                "coords": rota_coords,
                "distancia_m": distancia_rota
            })
            distancia_total_otimizada += distancia_rota
            
    return resultados, distancia_total_otimizada

# --- 3. FRONTEND ---

st.title("🚛 Otimizador de Rotas MVP")
st.markdown("**Regra:** O primeiro endereço da lista é sempre considerado a **Garagem/Depósito**.")

col_left, col_right = st.columns([1, 1.5])

with col_left:
    st.info("Passo 1: Cadastre os locais")
    input_end = st.text_input("Novo Endereço:", placeholder="Ex: Av Paulista, 1000, SP")
    if st.button("➕ Adicionar") and input_end:
        coords = buscar_coordenadas(input_end)
        if coords:
            st.session_state['locais'].append({"nome": input_end, "lat": coords[0], "lon": coords[1]})
            st.success("Adicionado!")
        else:
            st.error("Não achei. Tente colocar Cidade/Estado.")

    # Tabela de Locais
    if st.session_state['locais']:
        df = pd.DataFrame(st.session_state['locais'])
        st.dataframe(df, height=150, hide_index=True)
        if st.button("🗑️ Limpar Tudo"):
            st.session_state['locais'] = []
            st.rerun()

    st.write("---")
    st.info("Passo 2: Configurar e Calcular")
    n_veiculos = st.slider("Quantos veículos?", 1, 5, 1)
    
    # BOTÃO PRINCIPAL
    if st.button("🚀 OTIMIZAR AGORA", type="primary"):
        if len(st.session_state['locais']) < 2:
            st.warning("Adicione pelo menos 2 locais (Depósito + 1 Cliente).")
        else:
            with st.spinner("A IA está calculando as rotas..."):
                # 1. Calcular Otimizado
                rotas_finais, dist_otimizada = resolver_otimizacao(st.session_state['locais'], n_veiculos)
                
                # 2. Calcular "Jeito Burro" (Sequencial) para comparar
                dist_original = calcular_rota_original_sequencial(st.session_state['locais'])
                
                # Guardar resultado na sessão para não sumir
                st.session_state['resultado'] = {
                    'rotas': rotas_finais,
                    'dist_otimizada': dist_otimizada,
                    'dist_original': dist_original
                }

# --- EXIBIÇÃO DE RESULTADOS (FORA DAS COLUNAS PARA OCUPAR LARGURA TOTAL) ---

if 'resultado' in st.session_state and st.session_state['locais']:
    res = st.session_state['resultado']
    
    st.write("---")
    st.subheader("📊 Relatório de Performance")
    
    # Métricas de Economia
    c1, c2, c3 = st.columns(3)
    c1.metric("Distância Original (Sequencial)", f"{res['dist_original']/1000:.1f} km")
    c2.metric("Distância Otimizada (IA)", f"{res['dist_otimizada']/1000:.1f} km")
    
    economia = res['dist_original'] - res['dist_otimizada']
    c3.metric("Economia Estimada", f"{economia/1000:.1f} km", delta_color="normal")

    # Mapa e Texto lado a lado
    r_col1, r_col2 = st.columns([1, 1])
    
    with r_col1:
        st.write("### 📝 Detalhes das Rotas")
        for rota in res['rotas']:
            with st.expander(f"🚛 Veículo {rota['veiculo']} ({rota['distancia_m']/1000:.1f} km)", expanded=True):
                # Formatar a lista de passos com setinhas
                html_steps = ""
                for i, passo in enumerate(rota['passos']):
                    icone = "🏠" if (i==0 or i==len(rota['passos'])-1) else f"{i}."
                    html_steps += f"**{icone}** {passo}<br>⬇️<br>"
                
                st.markdown(html_steps[:-8], unsafe_allow_html=True) # Remove ultimo setinha
                
    with r_col2:
        # Mapa Resultante
        m = folium.Map(location=[st.session_state['locais'][0]['lat'], st.session_state['locais'][0]['lon']], zoom_start=12)
        colors = ['red', 'blue', 'green', 'purple', 'orange']
        
        for rota in res['rotas']:
            cor = colors[(rota['veiculo']-1) % len(colors)]
            
            # Linha da rota
            folium.PolyLine(rota['coords'], color=cor, weight=5, opacity=0.8).add_to(m)
            
            # Marcadores numerados
            for i, coord in enumerate(rota['coords']):
                 # Ignora ultimo ponto (que é o retorno ao deposito) para não duplicar marker
                if i < len(rota['coords']) - 1:
                    folium.Marker(
                        coord, 
                        popup=rota['passos'][i],
                        icon=folium.Icon(color=cor, icon="cloud" if i==0 else "info-sign")
                    ).add_to(m)

        st_folium(m, width="100%", height=500)

elif st.session_state['locais']:
    # Mostrar mapa vazio se ainda não calculou
    with col_right:
        m_start = folium.Map(location=[st.session_state['locais'][0]['lat'], st.session_state['locais'][0]['lon']], zoom_start=12)
        for loc in st.session_state['locais']:
            folium.Marker([loc['lat'], loc['lon']], popup=loc['nome']).add_to(m_start)
        st_folium(m_start, width="100%", height=400)
