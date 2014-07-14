%number of features
n=4;

%features from wpi localization spreadsheet (ENU Frame)
wpi1003 = [51.12 -16.44];
wpi1002 = [60.88 -29.33];
wpi1020 = [60.33 40.72];
wpi1021 = [70.97 40.23];

%features measured from generated pointcloud
map1003 = [-20.92 11.776];
map1002 = [-14.318 -1.7205];
map1020 = [-2.818 66.86];
map1021 = [7.32 64.65];

%concat corresponding features
B = [wpi1003;wpi1002;wpi1020;wpi1021];
A = [map1003;map1002;map1020;map1021];

[ret_R, ret_t] = rigid_transform_3D(A, B);

A2 = (ret_R*A') + repmat(ret_t, 1, n);
A2 = A2'
A2
B

ret_R
ret_t

% Find the error
err = A2 - B;
err = err .* err;
err = sum(err(:));
rmse = sqrt(err/n);

disp(sprintf("RMSE: %f", rmse));

%}


