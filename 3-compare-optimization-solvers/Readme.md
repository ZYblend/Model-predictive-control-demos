## Test 4 optimization solvers based on equality constrained quadratic program:
- "quadprog" provide by matlab
- First-order condition based closed-form solution
- projected gradient descent
- projected gradient descent with augmented lagrange <br>
### The last three are shown below:
![photo_2022-04-11_14-53-12](https://user-images.githubusercontent.com/36635562/162809556-0ad73c02-3fd2-419b-ac75-726213454634.jpg)
![photo_2022-04-11_14-53-14](https://user-images.githubusercontent.com/36635562/162810140-8ab14d26-6453-4671-9876-29dfdbca8cfa.jpg)


## Results and Conclusions:
100 iterations upper-bound is set for gradient descent algorithms.
- solver time: <br>
![image](https://user-images.githubusercontent.com/36635562/162810297-6c4deae8-bfca-48a8-8d66-99c12dfc4969.png) <br>
  1. The "quadprog" function takes the most time.
  2. The closed-form solution takes the less time. This is obvious, but if we add inequality constraints, no closed-form solution anymore.
  3. The projected gradient descent perform the best, because it will not take too much time and can be generalized with inequality constraints.
  4. Adding augmented lagrange is not necessary for a convex original problem, it will make solver more complex and take more time.

- All optimal solutions are same: <br>
![image](https://user-images.githubusercontent.com/36635562/162811024-72b9f6bd-ffe9-46ad-be11-5be44095a084.png)

