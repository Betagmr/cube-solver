import streamlit as st
import requests


def main():
    st.title("My Streamlit App")
    
    # Text field
    user_input = st.text_input("Enter your name:")
    
    # Button
    if st.button("Submit"):
        with st.spinner("waiting"):
            value = requests.get(f"http://localhost:8000/resolutor/{user_input}").json()
            st.write(value)


if __name__ == "__main__":
    main()
