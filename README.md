Pathfinder visualizer in Python

Supports <b>DFS</b>, <b>BFS</b>, <b>A*</b> and <b>Dijkstra</b>
<img src="https://github.com/andrazvrecko/pathFinder/blob/master/demoImg.png" width="500" height="500" />


<b>Use:</b>

  ```
  #Will create 50x50 Grid in a 1000x1000 window
  program = Program(GRID_SIZE=50, WIDTH=1000, HEIGHT=1000)
  program.startProgram()
  ```

  Right click - select start and end
  
  Left click - create walls
  
  H - Start DFS
  
  J - Start BFS
  
  K - Start A*
  
  L - Start Dijkstra
  
  C - Reset grid
  
  V - Reset and keep walls
  
  A - Generate Maze using Prims Algorithm
