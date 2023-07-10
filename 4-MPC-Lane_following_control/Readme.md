## Unconstrained path tracking
### Problem setup
Please refer to https://github.com/ZYblend/Model-predictive-control-demos/tree/main/3-MPC-Lane_following_control. <br>
I start from the lateral error dynamics. The goal is to **regulate the errors to zeros but don't want the steering angle change too fast (rate constraint) or exceed the saturation (box constraint)**.

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
Assume the path is approximated by second-order polynominals: <br>
<img src="https://user-images.githubusercontent.com/36635562/153495628-2bb9a53a-87b6-43a5-acd7-b8f8b4f30986.jpg" width="500" />


### Control Design
<img src="https://user-images.githubusercontent.com/36635562/162812214-3706d096-79f0-4eed-82f8-866edf0c92d0.jpg" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/162812226-9dfc833d-a854-4d7d-b27c-9ed19870af41.jpg" width="500" />
<img src="https://user-images.githubusercontent.com/36635562/162812232-850a7b52-7499-401c-a013-0584293f1b7c.jpg" width="500" />

### simulations
In simulation, I implement two different solver for my constraint path-tracking MPC:
- based on matlab function "fmincon"
- heavy-ball projected gradient descent 

#### Procedures:
- Run **"run_model_MPC.m"**: this file will (1) prepare system dynamics parameter; (2) define global trajectory in map; (3) prepare MPC parameters.
- Run experiment with **"fmincon"**: uncomment the line 11, 15 in **"microNole_MPC2021.slx/Latera Controller_fmincon1"**, and comment line 12,16. <br>
![image](https://user-images.githubusercontent.com/36635562/162813323-91a37fd6-f3d3-4110-8a15-6b16a84b869f.png) <br>
The relevant algorithm files: **"constrained_MPC_quaprog.m"**.
- Run experiment with **heavy-ball projected gradient descent**: uncomment the line 12,16 in **"microNole_MPC2021.slx/Latera Controller_fmincon1"**, and comment line 11,15 <br>
The relevant algorithm files: **"constrained_MPC_pgdhb.m"**, **"Heavy_ball_projected_Gradient_descent.m"**, **"Orthogonal_proj_onto_multiConvexSet.m"**.
- Run **"plotting_file.m"** after runing the simulink model.

#### Results:
The start of vehicle is (0,2), the trajectory start from (0,0).  <br>
<img src="https://user-images.githubusercontent.com/36635562/153500220-71bbe1e2-3722-4553-8dd6-da3f6095c899.png" width="500" /> <br>
![path_compare](https://user-images.githubusercontent.com/36635562/162815107-a0f97078-77bf-408d-a4da-87e4e3615b75.png)
![steering_compare](https://user-images.githubusercontent.com/36635562/162815157-cc29ffe8-cd39-4c4c-8afc-11aaaba26e36.png) <br>

Althought the unconstrained MPC can also trak the path, but at the middle when we change the target trajectory function, it reponse more than the constrained MPC because it does not have rate constriant on steering angle.



