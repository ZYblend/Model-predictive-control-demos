## Vehicle Dynamical model:
<img src="https://user-images.githubusercontent.com/36635562/153494946-36c0aa39-494a-4527-b6b8-9cf8c8f09d9c.png" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/153494978-f46271e6-7a1b-4294-a36d-4d33b360cc4f.png" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/153494987-39768a7c-8a1e-4db8-afc3-eee152576d93.png" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/153494999-59cef32f-e4fd-4916-9615-55ee1976c43f.png" width="500" />

## Lateral dynamical model:
<img src="https://user-images.githubusercontent.com/36635562/153495470-ac3715d2-f6b2-4d11-82b6-2070dd72007d.jpg" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/153495503-042200e6-3a5d-4bf7-b124-5092fa495378.jpg" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/153495510-afe308ea-8d63-4f11-a0cc-67a9d51dae7d.jpg" width="500" />

## Crosstrack error Calculation
Assume the path is approximated by second-order polynominals:
![crosstrack](https://user-images.githubusercontent.com/36635562/153495628-2bb9a53a-87b6-43a5-acd7-b8f8b4f30986.jpg)


## Controller design:
- longitudinal velocity cruise control: PI controller
- lane following control: MPC regulation control, Please refer to another folder [Optimal regulation control](https://github.com/ZYblend/Model-predictive-control-demos/tree/main/1-Optimal-Control)

## Simulation 1: Single lane following
Process:
- Run **run_model_MPC.m**: initialize the simulation variable, define trajectory, solve MPC control gain
- Run **microNole_MPC.slx**: lane following simulation
- Run **plotiing_file.m**: plot all results for you <br>

#### Stright lane tracking
![image](https://user-images.githubusercontent.com/36635562/153496856-711cfec5-2884-4b8b-b063-8462bce5dc5f.png)
#### counter clockwise tracking
Want to try other trajectory?
- Please go to **Trajectory_generator.m**, comment the line defineing "straight line" function, then uncomment the line defining "quandratic curve", shown below: <br>
![image](https://user-images.githubusercontent.com/36635562/153497244-4160c56f-4909-4742-955f-b0af8d65f86d.png) <br>
![image](https://user-images.githubusercontent.com/36635562/153497605-2ee10267-74a2-4615-89e5-911e72d4a54c.png) <br>

See steady error?
- Please go to 

#### clockwise tracking

#### Hybrid tracking

## Simulation 2: 
I use two second-order polynominal path fit a sin cruve:
