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
st.set_page_config(page_title="Otimizador de Rotas Web", layout="wide")

# Inicializar "memória" da sessão para guardar os endereços
if 'locais' not in st.session_state:
    st.session_state['locais'] = []

# --- 1. FUNÇÕES DE SUPORTE (GEOCODING & MATH) ---

def buscar_coordenadas(endereco):
    """Converte texto em (lat, long) usando OpenStreetMap"""
    geolocator = Nominatim(user_agent="meu_app_rotas_mvp_v1")
    try:
        location = geolocator.geocode(endereco)
        if location:
            return location.latitude, location.longitude
        return None
    except:
        return None

def calcular_distancia_haversine(lat1, lon1, lat2, lon2):
    """Matemática para calcular distância em metros"""
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    # Fator 1.4 para compensar tortuosidade das ruas
    return int(2 * R * math.asin(math.sqrt(a)) * 1.4)

# --- 2. SOLVER (OR-TOOLS) ---
def resolver_rota(lista_locais, num_veiculos):
    # Preparar Matriz
    tamanho = len(lista_locais)
    matriz = [[0] * tamanho for _ in range(tamanho)]
    
    for i in range(tamanho):
        for j in range(tamanho):
            if i != j:
                matriz[i][j] = calcular_distancia_haversine(
                    lista_locais[i]['lat'], lista_locais[i]['lon'],
                    lista_locais[j]['lat'], lista_locais[j]['lon']
                )
    
    # Configurar OR-Tools
    manager = pywrapcp.RoutingIndexManager(tamanho, num_veiculos, 0) # 0 é o depósito
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return matriz[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Adicionar dimensão de distância para equilibrar (opcional)
    routing.AddDimension(transit_callback_index, 0, 3000000, True, 'Distance')
    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_parameters)
    
    rotas = []
    if solution:
        for vehicle_id in range(num_veiculos):
            index = routing.Start(vehicle_id)
            rota_atual = []
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                rota_atual.append(lista_locais[node_index])
                index = solution.Value(routing.NextVar(index))
            
            # Adicionar o retorno ao depósito
            node_index = manager.IndexToNode(index)
            rota_atual.append(lista_locais[node_index]) 
            rotas.append(rota_atual)
            
    return rotas

# --- 3. INTERFACE VISUAL (STREAMLIT) ---

st.title("🚛 Otimizador de Rotas (Endereço Real)")
st.markdown("Adicione endereços. O primeiro será considerado o **DEPÓSITO** (Ponto de Partida).")

col1, col2 = st.columns([1, 2])

with col1:
    # Input de Endereço
    endereco_input = st.text_input("Digite o endereço completo:", placeholder="Ex: Av. Paulista, 1578, São Paulo")
    
    if st.button("Adicionar à Lista"):
        if endereco_input:
            with st.spinner('Buscando coordenadas...'):
                coords = buscar_coordenadas(endereco_input)
                if coords:
                    st.session_state['locais'].append({
                        "nome": endereco_input,
                        "lat": coords[0],
                        "lon": coords[1]
                    })
                    st.success(f"Encontrado: {coords}")
                else:
                    st.error("Endereço não encontrado. Tente adicionar cidade e estado.")

    # Lista de Pontos
    st.write("### 📍 Pontos a visitar:")
    if len(st.session_state['locais']) > 0:
        df_locais = pd.DataFrame(st.session_state['locais'])
        st.dataframe(df_locais[['nome', 'lat', 'lon']], hide_index=True)
        
        if st.button("Limpar Lista"):
            st.session_state['locais'] = []
            st.rerun()

    # Configuração da Frota
    num_veiculos = st.slider("Número de Veículos", 1, 5, 1)
    botao_calcular = st.button("🚀 Calcular Melhor Rota", type="primary")

with col2:
    # Mapa Inicial (ou Resultado)
    m = folium.Map(location=[-23.5505, -46.6333], zoom_start=11)
    
    # Desenhar pontos existentes
    for i, local in enumerate(st.session_state['locais']):
        icone = "home" if i == 0 else "info-sign"
        cor = "black" if i == 0 else "blue"
        folium.Marker(
            [local['lat'], local['lon']], 
            popup=local['nome'],
            icon=folium.Icon(color=cor, icon=icone)
        ).add_to(m)

    # SE CLICOU EM CALCULAR
    if botao_calcular and len(st.session_state['locais']) > 1:
        with st.spinner('Otimizando rotas...'):
            rotas_finais = resolver_rota(st.session_state['locais'], num_veiculos)
            
            cores = ['red', 'green', 'blue', 'orange', 'purple']
            
            for i, rota in enumerate(rotas_finais):
                coords_rota = [[p['lat'], p['lon']] for p in rota]
                
                # Desenhar linha da rota
                folium.PolyLine(
                    coords_rota, 
                    color=cores[i % len(cores)], 
                    weight=5, 
                    opacity=0.8,
                    tooltip=f"Veículo {i+1}"
                ).add_to(m)
                
                st.toast(f"Veículo {i+1}: {len(rota)-2} entregas agendadas.")

    st_folium(m, width="100%", height=600)
