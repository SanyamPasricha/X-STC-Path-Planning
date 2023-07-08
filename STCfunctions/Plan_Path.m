function [path,nodeMatrix,VEmatrix] = newSTC(path, originalMap, map, start_cellxy, nodeMatrix, VEmatrix, start_node, curr_node, prev_node)
    % When the robot is starting out from start_cell
    if (isempty(path))
        path = start_cellxy;
        res = map.Resolution;
        D = 1/res;
        ylimits = map.YWorldLimits;
        cellxy = start_cellxy;      %%% CHANGE TO BE MADE HERE
        [x,y] = node2xy(start_node,D,ylimits(2));
        nodexy = [x,y];
        diff = (cellxy - nodexy).*(2/D);
        % Hard coding for all the four possible directions to find the
        % prev_node and hence the direction robot is facing such that the 
        % robot is in bottom right corner of the major cell
        if (diff == [1 ,-1]) 
            prev_node = [curr_node(1) + 1,curr_node(2)];
        elseif (diff == [1, 1])
            prev_node = [curr_node(1),curr_node(2) + 1];
        elseif (diff == [-1, 1])
            prev_node = [curr_node(1) - 1,curr_node(2)];
        elseif (diff == [-1, -1])
            prev_node = [curr_node(1),curr_node(2) - 1];
        end
        %Finding the right, front and left node
        left = findLeft(prev_node,curr_node);
        right = findRight(prev_node,curr_node);
        front = findFront(prev_node,curr_node);
        right_check = check(right,nodeMatrix);
        front_check = check(front,nodeMatrix);
        left_check = check(left,nodeMatrix);
        prev_check = check(prev_node,nodeMatrix);%%% CHANGE TO BE MADE HERE
        % Checking the right,left and front node if they are free
        if right_check && isFree(right,map)
                nodeMatrix = createSTedge(nodeMatrix,curr_node,right);
                path = createPath(map,path,"right",prev_node,curr_node);
                prev_node = curr_node;
                curr_node = right;
                disp("right_start");
        elseif right_check && isVEpossible(originalMap,map,curr_node,right)
                VEmatrix = createVE(VEmatrix,curr_node,right);
                path = createObstaclePath(originalMap,map,path,"right",prev_node,curr_node);
                prev_node = right;
                disp("right Virtual");
            %disp("right");
        elseif front_check && isFree(front,map)
                nodeMatrix = createSTedge(nodeMatrix,curr_node,front);
                path = createPath(map,path,"front",prev_node,curr_node);
                prev_node = curr_node;
                curr_node = front;
                disp("front start");
        elseif front_check && isVEpossible(originalMap,map,curr_node,front)
                VEmatrix = createVE(VEmatrix,curr_node,front);
                path = createObstaclePath(originalMap,map,path,"front",prev_node,curr_node);
                prev_node = front;
                disp("front virtual");
            %disp("front");
        elseif left_check && isFree(left,map)
                nodeMatrix = createSTedge(nodeMatrix,curr_node,left);
                path = createPath(map,path,"left",prev_node,curr_node);
                prev_node = curr_node;
                curr_node = left;
                disp("left start");
        elseif left_check && isVEpossible(originalMap,map,curr_node,left)
                VEmatrix = createVE(VEmatrix,curr_node,left);
                path = createObstaclePath(originalMap,map,path,"left",prev_node,curr_node);
                prev_node = left;
                disp("left virtual");
            %disp("left");
        elseif prev_check && isFree(prev_node,map)
                nodeMatrix = createSTedge(nodeMatrix,curr_node,prev_node);
                path = createPath(map,path,"U-turn",prev_node,curr_node);
                temp = prev_node;
                prev_node = curr_node;
                curr_node = temp;
                disp("U-turn start");
        elseif prev_check && isVEpossible(originalMap,map,curr_node,prev_node)
                VEmatrix = createVE(VEmatrix,curr_node,prev_node);
                path = createObstaclePath(originalMap,map,path,"U-turn",prev_node,curr_node);
                disp("U-turn Virtual");
            %disp("U-turn");
        % There is no free major cell and only cell to be covered is the
        % start cell
        else
            path = completePath(map,path);
            disp("No Path");
            return;
        end

    else
        left = findLeft(prev_node,curr_node);
        right = findRight(prev_node,curr_node);
        front = findFront(prev_node,curr_node);
        right_check = check(right,nodeMatrix);
        front_check = check(front,nodeMatrix);
        left_check = check(left,nodeMatrix);
        
        if  right_check && (isFree(right,map))
            nodeMatrix = createSTedge(nodeMatrix,curr_node,right);

        elseif right_check && (~isVE(VEmatrix,curr_node,right)) && isVEpossible(originalMap,map,curr_node,right)
            VEmatrix = createVE(VEmatrix,curr_node,right);

        elseif front_check && (isFree(front,map))
            nodeMatrix = createSTedge(nodeMatrix,curr_node,front);

        elseif front_check && (~isVE(VEmatrix,curr_node,front)) && isVEpossible(originalMap,map,curr_node,front)
            VEmatrix = createVE(VEmatrix,curr_node,front);
        
        elseif left_check && (isFree(left,map))
            nodeMatrix = createSTedge(nodeMatrix,curr_node,left);
        
        elseif left_check && (~isVE(VEmatrix,curr_node,left)) && isVEpossible(originalMap,map,curr_node,left)
            VEmatrix = createVE(VEmatrix,curr_node,left);
        
        elseif (start_node == curr_node)
                path = completePath(map,path);
                return;  
        end
        if isSTedge(nodeMatrix,curr_node,right)
            path = createPath(map,path,"right",prev_node,curr_node);
            prev_node = curr_node;
            curr_node = right;
            disp("right");
        elseif isVE(VEmatrix,curr_node,right)
            path = createObstaclePath(originalMap,map,path,"right",prev_node,curr_node);
            prev_node = right;
            disp("right Virtual");
        elseif isSTedge(nodeMatrix,curr_node,front)
            path = createPath(map,path,"front",prev_node,curr_node);
            prev_node = curr_node;
            curr_node = front;
            disp("front");
        elseif isVE(VEmatrix,curr_node,front)
            path = createObstaclePath(originalMap,map,path,"front",prev_node,curr_node);
            prev_node = front;
            disp("front Virtual");
        elseif isSTedge(nodeMatrix,curr_node,left)
            path = createPath(map,path,"left",prev_node,curr_node);
            prev_node = curr_node;
            curr_node = left;
            disp("left");
        elseif isVE(VEmatrix,curr_node,left)
            path = createObstaclePath(originalMap,map,path,"left",prev_node,curr_node);
            prev_node = left;
            disp("left Virtual");
        else  
            path = createPath(map,path,"U-turn",prev_node,curr_node);
            temp = prev_node;
            prev_node = curr_node;
            curr_node = temp;
            disp("U-turn");
        end
    end
    %disp("Next call");
    [path,nodeMatrix,VEmatrix] = newSTC(path, originalMap, map, start_cellxy, nodeMatrix, VEmatrix, start_node, curr_node, prev_node);
end

function mat = createSTedge(nodeMatrix,node1,node2)
    nodeMatrix{node1(1),node1(2)} = [nodeMatrix{node1(1),node1(2)} ; node2];
    nodeMatrix{node2(1),node2(2)} = [nodeMatrix{node2(1),node2(2)} ; node1];
    mat = nodeMatrix;
end
function mat = createVE(VEmatrix,node1,node2)
    VEmatrix{node1(1),node1(2)} = [VEmatrix{node1(1),node1(2)} ; node2];
    %VEmatrix{node2(1),node2(2)} = [VEmatrix{node2(1),node2(2)} ; node1];
    mat = VEmatrix;
end
function isedge = isSTedge(nodeMatrix,node1,node2)
    if isempty(nodeMatrix{node1(1),node1(2)})
        isedge = 0;
    else
        isedge = any( all( nodeMatrix{node1(1),node1(2)} == node2 , 2) );
    end
end
function isedge = isVE(VEmatrix,node1,node2)
    if isempty(VEmatrix{node1(1),node1(2)})
        isedge = 0;
    else
        isedge = any( all( VEmatrix{node1(1),node1(2)} == node2 , 2) );
    end
end
function path = createPath(map,path,direction,prev_node,curr_node)   
    curr_cellxy = path(end,:);
    curr_cell = world2grid(map,curr_cellxy);
    prev_cell = curr_cell + (prev_node - curr_node);
    prev_cellxy = grid2world(map,prev_cell);
    if (direction == "right")
        path = [path ; Right(map,prev_cellxy,path(end,:))];
    elseif (direction == "front")
        path = [path ; Front(map,prev_cellxy,path(end,:))];
        path = [path ; Front(map,path(end-1,:),path(end,:))];
    elseif (direction == "left")
        path = [path ; Front(map,prev_cellxy,path(end,:))];
        path = [path ; Left(map,path(end-1,:),path(end,:))];
        path = [path ; Front(map,path(end-1,:),path(end,:))];
    elseif (direction == "U-turn")
        path = [path ; Front(map,prev_cellxy,path(end,:))];
        path = [path ; Left(map,path(end-1,:),path(end,:))];
        path = [path ; Left(map,path(end-1,:),path(end,:))];
        path = [path ; Front(map,path(end-1,:),path(end,:))];
    end
end
function checkvalue = check(node,nodeMatrix)
    sz = size(nodeMatrix);
    limitcond = ~any(node > sz | node <= 0);
    if (limitcond == 0)
        checkvalue = 0;
        return;
    end
    visitedcond = isnewNode(node,nodeMatrix);
    checkvalue = visitedcond;
end
function freenode = isFree(node,map)
    res = map.Resolution;
    D = 1/res;
    ylimits = map.YWorldLimits;
    [x,y] = node2xy(node,D,ylimits(2));
    occVal = checkOccupancy(map,[x + 0.5*D,y + 0.5*D ; x + 0.5*D,y - 0.5*D ; x - 0.5*D,y + 0.5*D ; x - 0.5*D,y - 0.5*D]);
    freenode = ~any(occVal);
end
function newnode = isnewNode(node,nodeMatrix)
    newnode = isempty(nodeMatrix{node(1),node(2)});
end
function path = completePath(map,path)
    while (any(path(end,:) ~= path(1,:)) || (size(path,1) == 1))
        curr_cell = world2grid(map,path(end,:));
        next_cell = grid2world(map,anticlockCell(curr_cell));
        path = [path ; next_cell];
    end
end
function nextcell = anticlockCell(cell)              %%% CHANGE TO BE MADE HERE
    row = cell(1);
    col = cell(2);
    rowbin = mod(row,2);
    colbin = mod(col,2);
    newrow = row + (rowbin + colbin - 1);
    newcol = col + (colbin - rowbin);
    nextcell = [newrow,newcol];
end

function possibility = isVEpossible(originalMap,map,curr_node,next_node)
    resD = map.Resolution;
    D = 1/resD;
    resd = originalMap.Resolution;
    d = 1/resd;
    [arr,arr1,arr2] = init_arr(map,curr_node,next_node,d,D);
    n = sum(checkOccupancy(originalMap,arr));
    len = size(arr,1);
    if (n >= len/2)
        possibility = 0;
        return;
    end
    occ1 = any(checkOccupancy(originalMap,arr1));
    occ2 = any(checkOccupancy(originalMap,arr2));
    possibility = ~(occ1 && occ2);
end
function [arr,arr1,arr2] = init_arr(map,curr_node,next_node,d,D)
    ylimits = map.YWorldLimits;
    [x,y] = node2xy(curr_node,D,ylimits(2));
    if (curr_node(1) == next_node(1))
        Y = (y - D + d/2):d:(y + D - d/2);
        X = ones(1,numel(Y))*(x + (D+d/2)*(next_node(2) - curr_node(2)));
        Y1 = (y - D/2 + d/2):d:y;
        X1 = ones(1,numel(Y1))*(x + (D+d/2)*(next_node(2) - curr_node(2)));
        Y2 = y:d:(y + D/2 - d/2);
        X2 = ones(1,numel(Y2))*(x + (D+d/2)*(next_node(2) - curr_node(2)));
    else
        X = (x - D + d/2):d:(x + D - d/2);
        Y = ones(1,numel(X))*(y + (D+d/2)*(curr_node(1) - next_node(1)));
        X1 = (x - D/2 + d/2):d:x;
        Y1 = ones(1,numel(X1))*(y + (D+d/2)*(curr_node(1) - next_node(1)));
        X2 = x:d:(x + D/2 - d/2);
        Y2 = ones(1,numel(X2))*(y + (D+d/2)*(curr_node(1) - next_node(1)));
    end
    arr = [X',Y'];
    arr1 = [X1',Y1'];
    arr2 = [X2',Y2'];
end