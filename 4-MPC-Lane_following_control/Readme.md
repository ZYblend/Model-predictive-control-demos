## Unconstrained path tracking
### Problem setup
Please refer to https://github.com/ZYblend/Model-predictive-control-demos/tree/main/3-MPC-Lane_following_control. <br>
I start from the lateral error dynamics. The goal is to **regulate the errors to zeros but don't want the steering angle change too fast (rate constraint) or exceed the saturation (box constraint)**.

### Deductions
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



