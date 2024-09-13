import gurobipy as gp
from gurobipy import GRB
import numpy as np


# Dados do problema
c =  [[0.0, 30.6465349354092, 19.288531839245117, 15.350694071965846, 25.610442326620834],
     [30.6465349354092, 0.0, 11.853178137302212, 45.996958993248725, 15.113230275628233],
     [19.288531839245117, 11.853178137302212, 0.0, 34.56446033244967, 10.417443502559175],
     [14.673016300682663, 24.904583595338995, 17.82385635846294, 0.0, 27.926055108941107],
     [20.441201012213252, 49.88792519395202, 39.27254470586952, 9.200855186650898, 0.0]] # matriz de distância (precisa ser ajustada para incluir todas as distâncias)

n = len(c)  # Número de cidades
q = 1050  # Capacidade do veículo (exemplo)
demanda = [0, 925, 1019, 572, 365]  # Exemplo de demandas

# Modelagem do problema CVRP com Gurobi
model = gp.Model()

# Definir o tempo limite para o solver
model.setParam(GRB.Param.TimeLimit, 120 * 60)  # Tempo limite de 120 minutos (em segundos)

# Variáveis de decisão: x_ij = 1 se o arco (i, j) for usado, 0 caso contrário
x = model.addVars(n, n, vtype=GRB.BINARY, name="x")

# Variáveis u_i (quantidade transportada após visitar a cidade i)
u = model.addVars(n, vtype=GRB.CONTINUOUS, name="u")

# Função objetivo: minimizar a distância total percorrida
model.setObjective(gp.quicksum(c[i][j] * x[i, j] for i in range(n) for j in range(n)), GRB.MINIMIZE)

# Restrições de rota: cada cidade deve ser visitada exatamente uma vez (exceto o depósito)
model.addConstrs(gp.quicksum(x[i, j] for j in range(n) if j != i) == 1 for i in range(1, n))
model.addConstrs(gp.quicksum(x[j, i] for j in range(n) if j != i) == 1 for i in range(1, n))

# Restrições de subtours (eliminação de subtours via MTZ)
model.addConstrs((u[i] - u[j] + q * x[i, j] <= q - demanda[j]) for i in range(1, n) for j in range(1, n) if i != j)

# Restrições de capacidade
model.addConstrs((u[i] >= demanda[i] for i in range(1, n)))
model.addConstrs((u[i] <= q for i in range(1, n)))

# Otimizar o modelo
model.optimize()

# Função para gerar as rotas a partir da solução
def generate_routes(edges, n):
    routes = []
    unvisited = set(range(1, n))  # Cidades a serem visitadas (excluindo o depósito)
    
    while unvisited:
        route = [0]  # Inicia no depósito
        current_city = 0
        while True:
            next_city = None
            for (i, j) in edges:
                if i == current_city and j in unvisited:
                    next_city = j
                    break
            if next_city is None:  # Se não houver mais cidades, retorne ao depósito
                break
            route.append(next_city)
            unvisited.remove(next_city)
            current_city = next_city
        route.append(0)  # Retorna ao depósito
        routes.append(route)
    
    return routes

# Coletando a solução
edges = []
if model.status == GRB.OPTIMAL:
    for i in range(n):
        for j in range(n):
            if x[i, j].x > 0.5:  # Se o arco (i, j) está na solução
                edges.append((i, j))

    routes = generate_routes(edges, n)

    # Imprimir as rotas dos caminhões
    for i, route in enumerate(routes):
        print(f"Caminhão {i + 1}: {' -> '.join(map(str, route))}")
if model.status == GRB.INFEASIBLE:
    model.computeIIS()
    model.write("infeasible_model.ilp")  # Salva o IIS em um arquivo
    print("Modelo é inviável. Verifique o arquivo 'infeasible_model.ilp' para mais detalhes.")

# Avaliação de desempenho
print(f"Tempo de execução: {model.Runtime} segundos")
print(f"Iterações: {model.IterCount}")
print(f"Nós do branch-and-bound: {model.NodeCount}")
print(f"Gap de relaxação linear: {model.MIPGap}")
print(f"Gap final: {model.MIPGap}")

