function h = plotcube2(ax, center, quat_orient, dim, color, alpha)

center = center(:)';
quat_orient = quat_orient(:)';

x_len = dim(1);
y_len = dim(2);
z_len = dim(3);

% Calculate the rotation matrix from the quaternion
R = quat2rotm(quat_orient);

% Calculate the corner points of the 3D box
halfWidth = x_len / 2;
halfHeight = y_len / 2;
halfDepth = z_len / 2;

corners = [-halfWidth, -halfHeight, -halfDepth;  % Bottom-left-back corner
           halfWidth, -halfHeight, -halfDepth;   % Bottom-right-back corner
           halfWidth, halfHeight, -halfDepth;    % Top-right-back corner
           -halfWidth, halfHeight, -halfDepth;   % Top-left-back corner
           -halfWidth, -halfHeight, halfDepth;   % Bottom-left-front corner
           halfWidth, -halfHeight, halfDepth;    % Bottom-right-front corner
           halfWidth, halfHeight, halfDepth;     % Top-right-front corner
           -halfWidth, halfHeight, halfDepth];   % Top-left-front corner

rotatedCorners = corners * R';  % Rotate the corners based on the quaternion
cornerPoints = rotatedCorners + center;  % Translate the corners to the center position

kwargs = {'FaceAlpha',alpha, 'EdgeAlpha',0.25 ,'HandleVisibility','off', 'Parent',ax};

% Plot the 3D box
hold(ax, 'on');
h = struct();
h.bottom = patch(cornerPoints([1, 2, 6, 5], 1), cornerPoints([1, 2, 6, 5], 2), cornerPoints([1, 2, 6, 5], 3), color, kwargs{:});  % Draw the bottom face
h.back = patch(cornerPoints([2, 3, 7, 6], 1), cornerPoints([2, 3, 7, 6], 2), cornerPoints([2, 3, 7, 6], 3), color, kwargs{:});  % Draw the back face
h.top = patch(cornerPoints([3, 4, 8, 7], 1), cornerPoints([3, 4, 8, 7], 2), cornerPoints([3, 4, 8, 7], 3), color, kwargs{:});  % Draw the top face
h.front = patch(cornerPoints([4, 1, 5, 8], 1), cornerPoints([4, 1, 5, 8], 2), cornerPoints([4, 1, 5, 8], 3), color, kwargs{:});  % Draw the front face
% Draw the remaining two side faces
h.side1 = patch(cornerPoints([1, 2, 3, 4], 1), cornerPoints([1, 2, 3, 4], 2), cornerPoints([1, 2, 3, 4], 3), color, kwargs{:});
h.side2 = patch(cornerPoints([5, 6, 7, 8], 1), cornerPoints([5, 6, 7, 8], 2), cornerPoints([5, 6, 7, 8], 3), color, kwargs{:});
