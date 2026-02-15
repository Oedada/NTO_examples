from fastapi import FastAPI
import pandas as pd
from typing import Optional
df = pd.read_csv("../../data/IMDB_Dataset.csv")
app = FastAPI()

# @app.get("/films")
# def get_all_films():
#     return f"Count of films: {df.size}"

@app.get("/films/{id}")
def get_film_from_id(id: int):
    return df.iloc[id]

@app.get("/films")
def get_film_from_id_p(id: Optional[int] = None):
    if id is None:
        return f"Count of films: {df.size}"
    return df.iloc[id]

@app.get("/")
def home():
    return "Hello"
# запустить с помощью коамнды uvicorn python.module.path.min:app --port 8888
