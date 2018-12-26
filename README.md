#  Extended Kalman Filter Project

## TODO

1. ~~compact and optimize~~
2. check why not working 
3. break down controller 
4. third party dir
5. precompiled
6. system model
7. measurement model
8. pimpl
9. go over https://github.com/mherb/kalman
10. polymorphism and inheritance
11. rename matrices and vectors to meaningful names
12. code documentation
13. write a project outline with overview of implementation steps
14. joseph form vs optimal kalman (standard form) ?
15. write project dir tree +  description of each
16. write expected input, output format and examples
17. write out relevant rubric metrics checklist
18. ~~Create reusable websocket module~~
19. make exploration, demo pipeline, viz notebook
20. results analysis
21. figure out interactive plots
22. make viz gifs
23. record intro video
24. put back install scripts and license



## Extended Kalman Filter

- numpy matricx multiply in left to right order, maintaining expected order of operations
- each sensor has its own update scheme
- update should be called once per sensor
- updates are asynchronous
- if two at same time: use both, careful of division by zero
- at first measurement: init and just set the mean
- we use a linear approximation of the non-linear conversion function of radar polar coordinate data
- For radar measurements, we need to map back to polar coordinates calculate the error `y`, that's what the Jacobian matrix is used for. 
- Radar measurement don't hold enough information to calculate `vx` and `vy`.



* Measurement update step via bayes rule multiplication. 

* Motion prediction step via done via total probability addition.

* Our priors are the measurement and motion variances

The measurement update step:
$$
y = z-H \cdot x \\
S = H \cdot P \cdot H^T + R \\
K = P \cdot H^T \cdot S^{-1} \\
x' = x + K \cdot y \\
P' = (I-K \cdot H) \cdot P
$$




and the prediction step:
$$
x' = F\cdot x+u  \\
P' = F \cdot P\cdot F^T \\
$$


where:

- x is estimate
- P is uncertainty covariance
- F is state transition
- u is motion vector
- z is measurement
- H is measurement function
- R is measurement noise
- I is identity matrix



The linear Taylor series expansion approximation (1st order) of a non-linear function:

$$
f(x) \approx f(\mu) + { \partial f(\mu) \over \partial x } \cdot (x - \mu)
$$

where:

- f(µ) is arctan(µ)
- ∂f(µ) is 1/(1+x^2)



Generalized to n dimensions:
$$
h(x')= \begin{pmatrix} \sqrt{ p{'}_x^2 + p{'}_y^2 }\\ \arctan(p_y' / p_x')\\ \frac{p_x' v_x' + p_y' v_y'}{\sqrt{p{'}_x^2 + p{'}_{y}^2}} \end{pmatrix}
$$

$$
T(x) = f(a) + (x - a)^TDf(a) + \frac 1{2!}(x-a)^T{D^2f(a)}(x - a) + ...
$$

$Df(a)$ is called the Jacobian matrix and $D^2f(a)$ is called the Hessian matrix. We don't calculate the Hessian because we're approximating to the first order.

$$
T(x) = f(a) + (x - a)^TDf(a)
$$

We'll calculate only the Jacobian matrix, containing all the partial derivatives:

$$
\Large H_j = 
\begin{bmatrix} 
\frac{\partial \rho}{\partial p_x} & \frac{\partial \rho}{\partial p_y} & \frac{\partial \rho}{\partial v_x} & \frac{\partial \rho}{\partial v_y}\\ \frac{\partial \varphi}{\partial p_x} & \frac{\partial \varphi}{\partial p_y} & \frac{\partial \varphi}{\partial v_x} & \frac{\partial \varphi}{\partial v_y}\\ \frac{\partial \dot{\rho}}{\partial p_x} & \frac{\partial \dot{\rho}}{\partial p_y} & \frac{\partial \dot{\rho}}{\partial v_x} & \frac{\partial \dot{\rho}}{\partial v_y} 
\end{bmatrix}  \\
\Large = 
\begin{bmatrix} 
\frac{p_x}{\sqrt[]{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt[]{p_x^2 + p_y^2}} & 0 & 0\\ -\frac{p_y}{p_x^2 + p_y^2} & \frac{p_x}{p_x^2 + p_y^2} & 0 & 0\\ \frac{p_y(v_x p_y - v_y p_x)}{(p_x^2 + p_y^2)^{3/2}} & \frac{p_x(v_y p_x - v_x p_y)}{(p_x^2 + p_y^2)^{3/2}} & \frac{p_x}{\sqrt[]{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt[]{p_x^2 + p_y^2}}
\end{bmatrix}
$$



#### Root Mean Squared Error (RMSE)

$$
RMSE = \sqrt{ \sum(\hat x - x)^2 \over N}
$$



## Data

## `./data/obj_pose-laser-radar-synthetic-input.txt`

```
L	3.122427e-01	5.803398e-01	1477010443000000	6.000000e-01	6.000000e-01	5.199937e+00	0	0	6.911322e-03
R	1.014892e+00	5.543292e-01	4.892807e+00	1477010443050000	8.599968e-01	6.000449e-01	5.199747e+00	1.796856e-03	3.455661e-04	1.382155e-02
...
```

- timestamp is in microseconds
- fradar data is in polar coordinates



## Resources

- [kalman filter - wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)
- [RMSE - wikipedia](https://en.wikipedia.org/wiki/Root-mean-square_deviation)
- [project rubric](https://review.udacity.com/#!/rubrics/748/view)
- [starter code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)
- [simulator](https://github.com/udacity/self-driving-car-sim/releases/)
- [visualization tools](https://github.com/udacity/CarND-Mercedes-SF-Utilities)
- [Q&A video](https://www.youtube.com/watch?v=J7WK9gEUltM&feature=youtu.be)
- [uWebsockets Library](https://github.com/uNetworking/uWebSockets)
- [Eigen Library](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [JSON Library](https://github.com/nlohmann/json)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md)
- [Pointer to Implementation Pattern](https://en.cppreference.com/w/cpp/language/pimpl)

### Reference Projects

* https://github.com/mherb/kalman
* https://github.com/ndrplz/self-driving-car
* https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter
* https://github.com/jeremyfix/easykf
* https://github.com/mithi/fusion-ekf
* https://github.com/ethz-asl/ethzasl_msf
* https://github.com/NikolasEnt/Extended-Kalman-Filter
* https://github.com/jeremy-shannon/CarND-Extended-Kalman-Filter-Project

