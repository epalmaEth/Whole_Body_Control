function P = linear_interpolation(start, stop, N)
% @param start -> Starting point
% @param stop -> Ending point
% @param N -> Number of interpolation points
% @return P -> Matrix of all points that the leg should go through

steps = (stop - start) / N;

x = start(1):steps(1):stop(1);
y = start(2):steps(2):stop(2);
z = start(3):steps(3):stop(3);

if isempty(x)   % If start and stop have the same coordinates
  x = start(1).*ones(1, N+1);
end

if isempty(y) 
  y = start(2).*ones(1, N+1);
end

if isempty(z) 
  z = start(3).*ones(1, N+1);
end

P=[x' y' z'];

end