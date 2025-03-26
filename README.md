# Controle do Turtlesim - ROS 2 Humble
## Objetivo
O objetivo deste laboratório foi implementar um nó de controle para a tartaruga do Turtlesim. O controlador permite que a tartaruga se desloque de uma posição arbitrária para um objetivo especificado, minimizando o erro de posição.
## Execução
1. Compile
   
```bash 
colcon build
source install/setup.bash
```
2. Execute o TurtleSim
```bash 
ros2 run turtlesim turtlesim_node

```
3. Execute o nó de Controle
```bash 
ros2 run turtle_control_Mariana turtle_control

```
4. Publique um objetivo (posição) manualmente
```bash 
ros2 topic pub /goal geometry_msgs/msg/Pose2D \
"{x: 7.0, y: 7.0, theta: 0.0}"
```

5. Rode o nó que define novas posições
```bash 
ros2 run turtle_control_Mariana goal_manager
```
