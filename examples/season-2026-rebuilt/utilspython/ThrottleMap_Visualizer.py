import numpy as np
import streamlit as st
import plotly.graph_objects as go
from scipy.interpolate import CubicSpline, interp1d
import numpy.polynomial.polynomial as poly

# Função para gerar a curva fiel ao ThrottleMap (Java) === 

# ============================================================
# === Bibliotecas: pip install streamlit plotly numpy scipy ==
# === Execução: streamlit run utilspython/ThrottleMap_Visualizer.py ==============
# ============================================================

def generate_curve(input_values, output_values, mode="linear", degree=1):
    x = np.array(input_values)
    y = np.array(output_values)
    x_dense = np.linspace(x[0], x[-1], 400)

    if mode == "linear":
        f = interp1d(x, y, kind="linear", fill_value="extrapolate")
        y_dense = f(x_dense)

    elif mode == "spline":
        spline = CubicSpline(x, y, bc_type='natural') 
        y_dense = spline(x_dense)

    elif mode == "polynomial":
        coeffs = poly.polyfit(x, y, degree)
        y_dense = poly.polyval(x_dense, coeffs)

    else:
        raise ValueError("Modo inválido. Use 'linear', 'spline' ou 'polynomial'.")

    return x_dense, y_dense


# ============================================================
# === Interface gráfica (Streamlit) ==========================
# ============================================================
st.set_page_config(page_title="ThrottleMap Visualizer FRC", layout="wide")

st.title("🎯 ThrottleMap Visualizer – FRC Robotics")
st.markdown(
    """
Ferramenta interativa para visualizar e validar curvas de mapeamento usadas na biblioteca `ThrottleMap` do código FRC.

**Modos suportados:**
- `Linear` → mesmo comportamento do modo padrão (piecewise linear).
- `Spline Cúbico` → suave, equivalente ao `SplineInterpolator` no Java.
- `Polinomial` → ajuste global de grau N, usando `PolynomialCurveFitter`.
"""
)

# Entradas de dados
col1, col2 = st.columns(2)

with col1:
    input_str = st.text_input(
        "📥 inputValues (valores X)",
        value="0.0, 0.2, 0.5, 0.8, 1.0",
        help="Digite os valores separados por vírgula, em ordem crescente."
    )

with col2:
    output_str = st.text_input(
        "📤 outputValues (valores Y)",
        value="0.0, 0.1, 0.6, 0.9, 1.0",
        help="Digite os valores correspondentes de saída."
    )

# Conversão para listas numéricas
try:
    input_values = [float(v.strip()) for v in input_str.split(",")]
    output_values = [float(v.strip()) for v in output_str.split(",")]
except ValueError:
    st.error("⚠️ Verifique se todos os valores são números válidos separados por vírgulas.")
    st.stop()

if len(input_values) != len(output_values):
    st.error("⚠️ As listas inputValues e outputValues precisam ter o mesmo tamanho.")
    st.stop()

if len(input_values) < 2:
    st.error("⚠️ É necessário pelo menos dois pontos para gerar uma curva.")
    st.stop()

# Seleção de modo
mode = st.selectbox(
    "⚙️ Tipo de curva",
    options=["Linear", "Spline Cúbico", "Polinomial (grau N)"]
)

degree = 1
if "Polinomial" in mode:
    degree = st.slider("🎚️ Grau do polinômio (N)", min_value=1, max_value=6, value=3)

# Botão para gerar
if st.button("🚀 Gerar Curva"):
    # Determinar modo interno (igual ao Java)
    internal_mode = "linear"
    if "Spline" in mode:
        internal_mode = "spline"
    elif "Polinomial" in mode:
        internal_mode = "polynomial"

    # Gerar curva
    x_dense, y_dense = generate_curve(input_values, output_values, internal_mode, degree)

    # Criar figura
    fig = go.Figure()

    # Pontos originais
    fig.add_trace(go.Scatter(
        x=input_values, y=output_values,
        mode="markers+text",
        text=[f"({x:.2f}, {y:.2f})" for x, y in zip(input_values, output_values)],
        textposition="top center",
        marker=dict(size=10, color="red"),
        name="Pontos Originais"
    ))

    # Linha interpolada
    fig.add_trace(go.Scatter(
        x=x_dense, y=y_dense,
        mode="lines",
        name=f"Curva {mode}",
        line=dict(color="blue", width=3),
        hovertemplate="x=%{x:.3f}<br>y=%{y:.3f}"
    ))

    # Layout
    fig.update_layout(
        title=f"ThrottleMap – {mode} {'(grau '+str(degree)+')' if 'Polinomial' in mode else ''}",
        xaxis_title="Input Value",
        yaxis_title="Output Value",
        hovermode="closest",
        template="plotly_white",
        legend=dict(x=0.02, y=0.98),
        height=600
    )

    st.plotly_chart(fig, use_container_width=True)

    # Tabela de valores gerados
    st.subheader("📊 Tabela de pontos interpolados (amostra)")
    sample_points = np.linspace(input_values[0], input_values[-1], 15)
    if internal_mode == "polynomial":
        coeffs = poly.polyfit(input_values, output_values, degree)
        sample_values = poly.polyval(sample_points, coeffs)
    elif internal_mode == "spline":
        spline = CubicSpline(input_values, output_values)
        sample_values = spline(sample_points)
    else:
        f = interp1d(input_values, output_values)
        sample_values = f(sample_points)

    table_data = {"Input": sample_points, "Output": sample_values}
    st.dataframe(table_data, use_container_width=True)
