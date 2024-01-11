import cv2
import numpy as np
import streamlit as st
import requests
from detector import preparar_imagen, obtener_colores, obtener_resultado, recortar_cubo


def main():
    st.title("My Streamlit App")
    
    # Text field
    user_input = st.text_input("Enter your name:")
    
    # Button
    if st.button("Submit"):
        with st.spinner("waiting"):
            value = requests.get(f"http://localhost:8000/resolutor/{user_input}").json()
            st.write(value)


def render_placeholder():
    caras_cubo = []
    
    for col in st.columns(3):
        with col:
            caras_cubo.append(
                st.image(np.zeros((80, 80, 3), np.uint8), channels="RGB")
            )
            caras_cubo.append(
                st.image(np.zeros((80, 80, 3), np.uint8), channels="RGB")
            )

    for i in range(6):

        state_image = st.session_state.estado_caras[i]

        if state_image is not None:

            caras_cubo[i].image(state_image, channels="RGB")

    st.write("Resultado: " + st.session_state.estado_scanner)


def modo_escaner():
    st.title("Modo Escaner")
    st.sidebar.title("Caras")
    boton_resolver = st.sidebar.button("Resolver")

    lista_caras = ["Cara 1", "Cara 2", "Cara 3", "Cara 4", "Cara 5", "Cara 6"]
    caras = st.sidebar.selectbox("Caras", lista_caras)
    boton_registrar = st.sidebar.button("Registrar")

    frame_placeholder = st.image(np.zeros((900, 900, 3), np.uint8), channels="RGB")

    render_placeholder()

    cap = st.session_state.estado_cam

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            st.write("The video capture has ended.")
            break

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_sin_cuadrado = frame.copy()
        frame = cv2.rectangle(frame, (305, 155), (385, 235), (0, 255, 0), 2)
        frame = cv2.flip(frame, 1)
        frame_placeholder.image(frame, channels="RGB")

        if boton_registrar:
            index = lista_caras.index(caras)
            cubo_recortado = recortar_cubo(frame_sin_cuadrado)
            st.session_state.estado_caras[index] = cubo_recortado
            
            cv2.destroyAllWindows()
            st.experimental_rerun()
            # st.rerun()

        if boton_resolver:
            
            resultado = ""

            for i in st.session_state.estado_caras:   
                lista_colores = obtener_colores(preparar_imagen(i))
                cara = obtener_resultado(lista_colores)
                resultado += cara

            st.session_state.estado_scanner = resultado
            st.experimental_rerun()

def modo_control():
    st.title("Modo Control")


def main2():
    # Menu
    st.sidebar.title("Menu")
    # Cambio de página
    app_mode = st.sidebar.selectbox("Modos de operación", ["Escaner", "Control"])

    # Automatico
    if app_mode == "Escaner":
        modo_escaner()
    elif app_mode == "Control":
        modo_control()


if __name__ == "__main__":
    if "estado_caras" not in st.session_state:
        st.session_state.estado_caras = [None] * 6
    if "estado_cam" not in st.session_state:
        cap = cv2.VideoCapture(0)
        st.session_state.estado_cam = cap
    if "estado_scanner" not in st.session_state:
        st.session_state.estado_scanner = ''
    # if "estado_colores" not in st.session_state:
    #     st.session_state.estado_colores = [None] * 6

    main2()