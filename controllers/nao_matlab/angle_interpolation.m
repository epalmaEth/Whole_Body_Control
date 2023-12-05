function P = angle_interpolation(start, stop, N)
% @param start -> Starting angle
% @param stop -> Ending angle
% @param N -> Number of interpolation points
% @return P -> Matrix of all angles that the arm should go through

steps = (stop - start) / N;

P = start:steps:stop;

if isempty(P)   % If start and stop is the same angle
  P = start.*ones(1, N+1);
end

end