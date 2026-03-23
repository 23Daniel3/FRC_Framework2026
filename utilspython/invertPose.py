import math
import tkinter as tk
from tkinter import ttk, messagebox

FIELD_LENGTH = 16.54
FIELD_WIDTH = 8.07

def parse_val(s):
    try:
        return float(s.replace(",", "."))
    except:
        raise ValueError

def format_val(v):
    return f"{v:.3f}"

def normalize_theta(theta):
    t = ((theta + math.pi) % (2 * math.pi)) - math.pi
    return t

def get_inputs():
    try:
        x = parse_val(entry_x.get())
        y = parse_val(entry_y.get())
        theta = parse_val(entry_theta.get())
        return x, y, theta
    except:
        messagebox.showerror("Erro", "Valores inválidos. Use números (ex: 3.25 ou 3,25).")
        raise

def set_results(x, y, theta):
    entry_x.delete(0, tk.END); entry_x.insert(0, format_val(x))
    entry_y.delete(0, tk.END); entry_y.insert(0, format_val(y))
    entry_theta.delete(0, tk.END); entry_theta.insert(0, format_val(theta))
    label_result.config(text=f"x = {format_val(x)}  y = {format_val(y)}  θ = {format_val(theta)} rad")
    history.insert(0, f"x={format_val(x)}  y={format_val(y)}  θ={format_val(theta)}")
    if history.size() > 20:
        history.delete(20, tk.END)

def invert_x():
    try:
        x, y, theta = get_inputs()
    except:
        return
    nx = FIELD_LENGTH - x
    ntheta = normalize_theta(math.pi - theta)
    set_results(nx, y, ntheta)

def invert_y():
    try:
        x, y, theta = get_inputs()
    except:
        return
    ny = FIELD_WIDTH - y
    ntheta = normalize_theta(-theta)
    set_results(x, ny, ntheta)

def invert_both():
    try:
        x, y, theta = get_inputs()
    except:
        return
    nx = FIELD_LENGTH - x
    ny = FIELD_WIDTH - y
    ntheta = normalize_theta(math.pi - theta)
    ntheta = normalize_theta(-ntheta)
    set_results(nx, ny, ntheta)

def copy_result():
    r = label_result.cget("text")
    root.clipboard_clear()
    root.clipboard_append(r)
    root.update()
    messagebox.showinfo("Copiado", "Resultado copiado para a área de transferência.")

def load_history(event=None):
    sel = history.curselection()
    if not sel:
        return
    val = history.get(sel[0])
    parts = val.split()
    try:
        x = float(parts[0].split("=")[1])
        y = float(parts[1].split("=")[1])
        th = float(parts[2].split("=")[1])
        set_results(x, y, th)
    except:
        pass

def clear_inputs():
    entry_x.delete(0, tk.END)
    entry_y.delete(0, tk.END)
    entry_theta.delete(0, tk.END)
    label_result.config(text="")

root = tk.Tk()
root.title("Inverter Pose — Campo FRC")
root.geometry("520x360")
root.resizable(False, False)

style = ttk.Style(root)
style.configure("TFrame", background="#f4f6f8")
style.configure("TButton", padding=6)
style.configure("TLabel", background="#f4f6f8")
style.configure("Header.TLabel", font=("Segoe UI", 14, "bold"))
style.configure("Field.TEntry", padding=6, font=("Segoe UI", 11))

frame = ttk.Frame(root, padding=12)
frame.pack(fill="both", expand=True)

header = ttk.Label(frame, text="Inversor de Pose (Campo 16.54m × 8.07m)", style="Header.TLabel")
header.grid(row=0, column=0, columnspan=4, pady=(0,10), sticky="w")

ttk.Label(frame, text="X (m)").grid(row=1, column=0, sticky="w", padx=(0,6))
ttk.Label(frame, text="Y (m)").grid(row=2, column=0, sticky="w", padx=(0,6))
ttk.Label(frame, text="Theta (rad)").grid(row=3, column=0, sticky="w", padx=(0,6))

entry_x = ttk.Entry(frame, width=18, style="Field.TEntry")
entry_y = ttk.Entry(frame, width=18, style="Field.TEntry")
entry_theta = ttk.Entry(frame, width=18, style="Field.TEntry")

entry_x.grid(row=1, column=1, padx=(0,10), pady=4)
entry_y.grid(row=2, column=1, padx=(0,10), pady=4)
entry_theta.grid(row=3, column=1, padx=(0,10), pady=4)

btn_x = ttk.Button(frame, text="Inverter X", command=invert_x)
btn_y = ttk.Button(frame, text="Inverter Y", command=invert_y)
btn_both = ttk.Button(frame, text="Inverter X e Y", command=invert_both)
btn_copy = ttk.Button(frame, text="Copiar Resultado", command=copy_result)
btn_clear = ttk.Button(frame, text="Limpar", command=clear_inputs)

btn_x.grid(row=1, column=2, padx=6, pady=4, sticky="ew")
btn_y.grid(row=2, column=2, padx=6, pady=4, sticky="ew")
btn_both.grid(row=3, column=2, padx=6, pady=4, sticky="ew")
btn_copy.grid(row=4, column=2, padx=6, pady=(8,4), sticky="ew")
btn_clear.grid(row=5, column=2, padx=6, pady=4, sticky="ew")

label_result = ttk.Label(frame, text="", font=("Segoe UI", 11))
label_result.grid(row=4, column=0, columnspan=2, pady=(8,4), sticky="w")

history_label = ttk.Label(frame, text="Histórico (clique para carregar):")
history_label.grid(row=5, column=0, columnspan=2, sticky="w")

history = tk.Listbox(frame, height=8)
history.grid(row=6, column=0, columnspan=3, sticky="nsew", pady=(6,0))
history.bind("<Double-Button-1>", load_history)

scroll = ttk.Scrollbar(frame, orient="vertical", command=history.yview)
history.config(yscrollcommand=scroll.set)
scroll.grid(row=6, column=3, sticky="ns", pady=(6,0))

info = ttk.Label(frame, text="Use ',' ou '.' como separador decimal. Theta em radianos. Histórico guarda as últimas 20 entradas.", font=("Segoe UI", 8))
info.grid(row=7, column=0, columnspan=4, pady=(10,0), sticky="w")

root.mainloop()
