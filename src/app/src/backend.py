from fastapi import FastAPI
from cube_solver import CubeSolverService
from arm_controller import ArmMoveService

app = FastAPI()
cube_solver = CubeSolverService()
arm_controller = ArmMoveService()


@app.get("/resolutor/{input_string}")
def resolutor(input_string: str):
    result = cube_solver.get_cube_solution(input_string)
    return result

@app.get("/robot/{input_string}")
def robot(input_string: str):
    result = arm_controller.launch_movements(input_string)
    return result