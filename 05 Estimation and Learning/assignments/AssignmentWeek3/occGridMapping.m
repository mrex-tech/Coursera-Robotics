% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 

% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
numScans = size(scanAngles);
for j = 1:N % for each time,
    x = pose(1, j);
    y = pose(2, j);
    theta = pose(3, j);
    
    ix_orig = ceil(x*myResol)+myorigin(1);
    iy_orig = ceil(y*myResol)+myorigin(2);
      
    % Find grids hit by the rays (in the gird map coordinate)
    rays = ranges(:, j);
    x_occ =  rays .* cos(scanAngles + theta) + x;
    y_occ = -rays .* sin(scanAngles + theta) + y;
    
    ix_occ = ceil(x_occ * myResol) + myorigin(1);
    iy_occ = ceil(y_occ * myResol) + myorigin(2);

    % Find occupied-measurement cells and free-measurement cells
    occ = sub2ind(size(myMap), iy_occ, ix_occ);
    
    free = [];
    for k = 1:numScans
        [ix_free, iy_free] = bresenham(ix_orig, iy_orig, ix_occ(k), iy_occ(k));  
        free = [free; iy_free, ix_free];
    end
    free = sub2ind(size(myMap), free(:, 1), free(:, 2)); % Convert to 1d

    % Update the log-odds
    myMap(occ) = myMap(occ) + lo_occ;
    myMap(free) = myMap(free) - lo_free;

    % Saturate the log-odd values
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;

    % Visualize the map as needed
   

end

end

