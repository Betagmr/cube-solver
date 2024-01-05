from fastapi import FastAPI
from cube_solver import CubeSolverService

app = FastAPI()
cube_solver = CubeSolverService()

@app.get("/resolutor/{input_string}")
def resolutor(input_string: str):
    result = cube_solver.get_cube_solution(input_string)
    return result



