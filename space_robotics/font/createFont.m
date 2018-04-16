function font = createFont()
font = containers.Map;

font(' ') = {{[0,0],[1,0],[0,0,0]}};

font('A') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,0.55],[0,0,0]},{[1,0.55],[0,0.55],[0.5,0.55,-1]},...
            {[0,0.55],[0,0],[0,0,0]}};

font('B') = {{[0,0],[0.8,0],[0,0,0]},{[0.8,0],[0.8,0.5],[0.7,0.25,-1]},{[0.8,0.5],[0.8,1],[0.7,0.75,-1]},...
            {[0.8,1],[0,1],[0,0,0]}, {[0,1],[0,0],[0,0,0]}};

font('C') = {{[0.45,0],[1,0],[0,0,0]},{[1,0],[1,1],[0,0,0]},{[1,1],[0.45,1],[0,0,0]},...
            {[0.45,1],[0.45,0],[0.45,0.5,-1]}};

font('D') = {{[0,0],[0.45,0],[0,0,0]},{[0.45,0],[0.45,1],[0.45,0.5,-1]},{[0.45,1],[0,1],[0,0,0]},...
            {[0,1],[0,0],[0,0,0]}};

font('E') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,0.33],[0,0,0]},{[1,0.33],[0.66,0.33],[0,0,0]},...
            {[0.66,.33],[0.66,0.66],[0.66,0.495,-1]}, {[0.66,0.66],[1,0.66],[0,0,0]},...
            {[1,0.66],[1,1],[0,0,0]},{[1,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};

font('F') = {{[0,0],[0.66,0],[0,0,0]},{[0.66,0],[0.66,0.33],[0,0,0]},...
            {[0.66,.33],[0.66,0.66],[0.66,0.495,-1]}, {[0.66,0.66],[1,0.66],[0,0,0]},...
            {[1,0.66],[1,1],[0,0,0]},{[1,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};         

font('G') = {{[0.5,0],[1,0],[0,0,0]},{[1,0],[1,0.66],[0,0,0]},...
            {[1,0.66],[0.66,0.66],[0,0,0]}, {[0.66,0.66],[0.66,1],[0,0,0]},...
            {[0.66,1],[0.5,1],[0,0,0]},{[0.66,1],[0.5,1],[0,0,0]},{[0.5,1],[0.5,0],[0.5,0.5,-1]}};

font('H') = {{[0,0],[0.5,0.3542],[0,0.53,-1]},{[0.5,0.3542],[1,0],[1,0.53,-1]},...
             {[1,0],[1,1],[0,0,0]}, {[1,1],[0.5,0.6458],[1,0.47,-1]},...
             {[0.5,0.6458],[0,1],[0,0.47,-1]},{[0,1],[0,0],[0,0,0]}};

font('I') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,1],[0,0,0]},...
             {[1,1],[0,1],[0,0,0]}, {[0,1],[0,0],[0,0,0]}};

font('J') = {{[0,0],[0.66,0],[0,0,0]},{[0.66,0],[1,0.33],[0.66,0.33,-1]},...
             {[1,0.33],[1,1],[0,0,0]}, {[1,1],[0.33,1],[0,0,0]},{[0.33,1],[0.33,0.33],[0,0,0]},{[.33,.33],[0,.33],[0,0,0]},{[0,.33],[0,0],[0,0,0]}};

font('K') = {{[0,0],[1,0],[0,0,0]},{[1,0],[0.5,0.5],[0.5,0,-1]},...
             {[0.5,0.5],[1,1],[0.5,1,-1]}, {[1,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};

font('L') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,0.33],[0,0,0]},...
             {[1,0.33],[0.66,0.33],[0,0,0]}, {[0.66,0.33],[0.66,1],[0,0,0]},{[0.66,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};

font('M') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,1],[0,0,0]},...
             {[1,1],[0.5,0.5],[1,0.5,-1]}, {[0.5,0.5],[0,1],[0,0.5,-1]},{[0,1],[0,0],[0,0,0]}};

font('N') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,0.66],[0,0,0]},...
             {[1,0.66],[0.66,1],[0.66,0.66,-1]}, {[0.66,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};

font('O') = {{[0.5,0],[0.5,1],[0.5,0.5,-1]},{[0.5,1],[0.5,0],[0.5,0.5,-1]}};

font('P') = {{[0,0],[0.66,0],[0,0,0]},{[0.66,0],[0.66,0.33],[0,0,0]},...
             {[0.66,0.33],[1,0.33],[0,0,0]}, {[1,0.33],[1,1],[0,0,0]},{[1,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};

font('Q') = {{[0.5,0],[1,0],[0,0,0]},{[1,0],[1,0.5],[0,0,0]}, {[1,0.5],[0.5,0],[0.5,0.5,-1]}};

font('R') = {{[0,0],[1,0],[0,0,0]},{[1,0],[0.66,0.34],[0.66,0,-1]},...
             {[0.66,0.34],[0.66,1],[0.67,0.67,-1]}, {[0.66,1],[0,1],[0,0,0]},{[0,1],[0,0],[0,0,0]}};

font('S') = {{[0,0],[0.66,0],[0,0,0]},{[0.66,0],[0.66,0.66],[0.66,0.33,-1]},...
             {[0.66,0.66],[1,0.66],[0,0,0]}, {[1,0.66],[1,1],[0,0,0]},{[1,1],[0.66,1],[0,0,0]},...
             {[0.34,1],[0.34,0.34],[0.34,0.67,-1]},{[0.34,0.34],[0,0.34],[0,0,0]},{[0,0.34],[0,0],[0,0,0]}};


font('T') = {{[0.15,0],[0.85,0],[0,0,0]},{[0.85,0],[0.85,0.66],[0,0,0]},...
             {[0.85,0.66],[1,0.66],[0,0,0]}, {[1,0.66],[1,1],[0,0,0]},{[1,1],[0,1],[0,0,0]},...
             {[0,1],[0,0.66],[0,0,0]},{[0,0.66],[0.15,0.66],[0,0,0]},{[0.15,0.66],[0.15,0],[0,0,0]}};

font('U') = {{[0.5,0],[1,0.5],[0.5,0.5,-1]},{[1,0.5],[1,1],[0,0,0]},...
             {[1,1],[0,1],[0,0,0]}, {[0,1],[0,0.5],[0,0,0]},{[0,0.5],[0.5,0],[0.5,0.5,-1]}}; 

font('V') = {{[0.5,0],[1,0.5],[0.5,0.5,-1]},{[1,0.5],[1,1],[0,0,0]},...
             {[1,1],[0.5,0.5],[1,0.5,-1]}, {[0.5,0.5],[0,1],[0,0.5,-1]},{[0,1],[0,0.5],[0,0,0]},{[0,0.5],[0.5,0],[0.5,0.5,-1]}}; 

font('W') = {{[0,0],[0.5,0.3542],[0,0.53,-1]},{[0.5,0.3542],[1,0],[1,0.53,-1]},...
             {[1,0],[1,1],[0,0,0]}, {[1,1],[0,1],[0,0,0]}, {[0,1],[0,0],[0,0,0]}};

font('X') = {{[0,0],[1,0],[0,0,0]},{[1,0],[0.6458,0.5],[0.47,0,-1]},...
             {[0.6458,0.5],[1,1],[0.47,1,-1]}, {[1,1],[0,1],[0,0,0]},...
             {[0,1],[0.3542,0.5],[0.53,1,-1]},{[0.3542,0.5],[0,0],[0.53,0,-1]}};

font('Y') = {{[0.33,0],[1,0],[0,0,0]},{[1,0],[1,1],[0,0,0]},...
             {[1,1],[0,1],[0,0,0]}, {[0,1],[0,0.66],[0,0,0]},{[0,0.66],[0.33,0.33],[0.33,0.66,-1]},{[0.33,0.33],[0.33,0],[0,0,0]}};  

font('Z') = {{[0,0],[1,0],[0,0,0]},{[1,0],[1,0.33],[0,0,0]},...
             {[1,0.33],[0.66,0.33],[0,0,0]}, {[0.66,0.33],[0.99,0.66],[0.66,0.66,-1]},{[0.99,0.66],[0.99,1],[0,0,0]},{[0.99,1],[0,1],[0,0,0]},...
             {[0,1],[0,0.66],[0,0,0]},{[0,0.66],[0.33,0.66],[0,0,0]},{[0.33,0.66],[0,0.33],[0.33,0.33,-1]},{[0.33,0.66],[0,0],[0,0,0]}};                              
end