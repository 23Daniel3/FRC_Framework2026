import json
import math
import os
import tkinter as tk
from tkinter import filedialog

def compute_heading(waypoints, i, ideal_start, goal_end):
    """
    Calcula o heading (em graus) para o waypoint de índice i.
    - Se existir apenas um waypoint, usa ideal_start (se definido) ou 0.0.
    - No primeiro waypoint, se ideal_start estiver definido, usa-o; 
      caso contrário, calcula o ângulo entre o primeiro e o segundo ponto.
    - No último waypoint, se goal_end estiver definido, usa-o; 
      caso contrário, calcula o ângulo entre o penúltimo e o último ponto.
    - Para os intermediários, calcula o ângulo entre o ponto atual e o próximo.
    """
    n = len(waypoints)
    if n == 0:
        return 0.0
    if n == 1:
        return ideal_start if ideal_start is not None else 0.0
    if i == 0:
        if ideal_start is not None:
            return ideal_start
        else:
            x0 = waypoints[0]["anchor"]["x"]
            y0 = waypoints[0]["anchor"]["y"]
            x1 = waypoints[1]["anchor"]["x"]
            y1 = waypoints[1]["anchor"]["y"]
            return math.degrees(math.atan2(y1 - y0, x1 - x0))
    elif i == n - 1:
        if goal_end is not None:
            return goal_end
        else:
            x0 = waypoints[i-1]["anchor"]["x"]
            y0 = waypoints[i-1]["anchor"]["y"]
            x1 = waypoints[i]["anchor"]["x"]
            y1 = waypoints[i]["anchor"]["y"]
            return math.degrees(math.atan2(y1 - y0, x1 - x0))
    else:
        x0 = waypoints[i]["anchor"]["x"]
        y0 = waypoints[i]["anchor"]["y"]
        x1 = waypoints[i+1]["anchor"]["x"]
        y1 = waypoints[i+1]["anchor"]["y"]
        return math.degrees(math.atan2(y1 - y0, x1 - x0))

def convert_path_to_traj(file_path):
    """
    Lê um arquivo .path e gera um arquivo .traj com a seguinte estrutura:
    
    {
      "name": <nome>,
      "version": 1,
      "snapshot": {
         "waypoints": [ ... ],
         "constraints": [ ... ],
         "targetDt": 0.05
      },
      "params": {
         "waypoints": [ ... ],
         "constraints": [ ... ],
         "targetDt": {"exp": "0.05 s", "val": 0.05}
      },
      "trajectory": {
         "sampleType": null,
         "waypoints": [],
         "samples": [],
         "splits": []
      },
      "events": []
    }
    
    Os waypoints são extraídos do campo "anchor" de cada waypoint do .path;
    o heading é definido usando idealStartingState e goalEndState, se disponíveis, ou calculado.
    """
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            path_data = json.load(f)
    except Exception as e:
        print(f"Erro ao ler o arquivo {file_path}: {e}")
        return

    # Obtém rotações iniciais e finais, se definidas
    ideal_start = path_data.get("idealStartingState", {}).get("rotation", None)
    goal_end = path_data.get("goalEndState", {}).get("rotation", None)
    
    # Extração dos waypoints do arquivo .path
    waypoints_in = path_data.get("waypoints", [])
    if not waypoints_in:
        print(f"O arquivo {file_path} não contém waypoints.")
        return

    # Lista de intervalos padrão para atribuir aos waypoints (se houver menos, usamos 20)
    default_intervals = [21, 28, 23, 38, 25, 40]
    
    snapshot_waypoints = []
    params_waypoints = []
    for i, wp in enumerate(waypoints_in):
        anchor = wp.get("anchor", {})
        x = anchor.get("x", 0.0)
        y = anchor.get("y", 0.0)
        
        # Define o heading:
        # - Se for o primeiro e existir idealStartingState.rotation, usa-o.
        # - Se for o último e existir goalEndState.rotation, usa-o.
        # - Caso contrário, calcula com base nos pontos.
        if i == 0 and ideal_start is not None:
            heading = ideal_start
        elif i == len(waypoints_in) - 1 and goal_end is not None:
            heading = goal_end
        else:
            heading = compute_heading(waypoints_in, i, ideal_start, goal_end)
        
        interval = default_intervals[i] if i < len(default_intervals) else 20
        
        # Cria o waypoint para a seção snapshot
        snap_wp = {
            "x": x,
            "y": y,
            "heading": heading,
            "intervals": interval,
            "split": False,
            "fixTranslation": True,
            "fixHeading": True,
            "overrideIntervals": False
        }
        snapshot_waypoints.append(snap_wp)
        
        # Cria o waypoint para a seção params (com os valores "embrulhados")
        params_wp = {
            "x": {"exp": f"{x} m", "val": x},
            "y": {"exp": f"{y} m", "val": y},
            "heading": {"exp": f"{heading} deg", "val": heading},
            "intervals": interval,
            "split": False,
            "fixTranslation": True,
            "fixHeading": True,
            "overrideIntervals": False
        }
        params_waypoints.append(params_wp)
    
    # Define constraints padrão (copiadas do exemplo fornecido)
    snapshot_constraints = [
        {"from": "first", "to": None, "data": {"type": "StopPoint", "props": {}}, "enabled": True},
        {"from": "last", "to": None, "data": {"type": "StopPoint", "props": {}}, "enabled": True},
        {"from": "first", "to": "last", "data": {"type": "KeepInRectangle", "props": {"x": 0.0, "y": 0.0, "w": 16.54, "h": 8.21}}, "enabled": True}
    ]
    
    params_constraints = [
        {"from": "first", "to": None, "data": {"type": "StopPoint", "props": {}}, "enabled": True},
        {"from": "last", "to": None, "data": {"type": "StopPoint", "props": {}}, "enabled": True},
        {"from": "first", "to": "last", "data": {"type": "KeepInRectangle", "props": {
            "x": {"exp": "0 m", "val": 0.0},
            "y": {"exp": "0 m", "val": 0.0},
            "w": {"exp": "16.54 m", "val": 16.54},
            "h": {"exp": "8.21 m", "val": 8.21}
        }}, "enabled": True}
    ]
    
    snapshot = {
        "waypoints": snapshot_waypoints,
        "constraints": snapshot_constraints,
        "targetDt": 0.05
    }
    
    params = {
        "waypoints": params_waypoints,
        "constraints": params_constraints,
        "targetDt": {"exp": "0.05 s", "val": 0.05}
    }
    
    # Define o objeto final .traj; o nome é obtido do campo "folder" do .path ou do nome do arquivo
    output_data = {
        "name": path_data.get("folder", os.path.splitext(os.path.basename(file_path))[0]),
        "version": 1,
        "snapshot": snapshot,
        "params": params,
        "trajectory": {
            "sampleType": None,
            "waypoints": [],
            "samples": [],
            "splits": []
        },
        "events": []
    }
    
    output_dir = "C:/Users/Maker/Desktop/TechMaker_2025_Reefscape/src/main/deploy/choreo/"
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, os.path.splitext(os.path.basename(file_path))[0] + ".traj")
    try:
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(output_data, f, indent=2)
        print(f"Arquivo convertido salvo como: {output_file}")
    except Exception as e:
        print(f"Erro ao salvar {output_file}: {e}")

def main():
    # Inicializa a interface Tkinter (sem exibir a janela principal)
    root = tk.Tk()
    root.withdraw()
    file_paths = filedialog.askopenfilenames(
        title="Selecione arquivos .path",
        filetypes=[("Arquivos PathPlanner", "*.path")]
    )
    if not file_paths:
        print("Nenhum arquivo selecionado.")
        return
    for file_path in file_paths:
        print(f"Convertendo: {file_path}")
        convert_path_to_traj(file_path)

if __name__ == "__main__":
    main()
