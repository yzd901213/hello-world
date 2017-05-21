function path = Astar(Map, snode, enode)
%%
% A*算法主要适用于网格化中求解最优路径，因此与相邻的点存在路径权值，其他点不存在
% A：存储各节点之间的距离矩阵
% snode：起始节点
% enode：结束节点

% 调用测试
if nargin == 0
    Astar_Test();
    return;
end

%配合MixedPathplanning
Map(snode.x, snode.y) = 'S';
Map(enode.x, enode.y) = 'E';


[m, n] = size(Map);
H = Inf*ones(m, n); % 存储当前点到目标点的路径代价估计
G = zeros(m, n); % 存储起始点到当前点的路径代价实际值
F = Inf*ones(m, n); % 起始点经过当前点到目标点的代价
pnode.x = 0;
pnode.y = 0;
P = repmat(pnode,[m, n]); % 该点的父节点

VISITED = '-'; % 被访问过
WALL = '2'; %         W
START = 'S'; % 起点
SPACE = '0'; % 空地    .
ON_PATH = '*'; % 在结果路径上
END = 'E'; % 终点

directs = [[1, 0]; [-1, 0]; [0, 1]; [0, -1]];

%%
% 设置起始点
H(snode.x, snode.y) = abs(enode.x - snode.x) + abs(enode.y - snode.y);
F(snode.x, snode.y) = G(snode.x, snode.y) + H(snode.x, snode.y);

Open = [snode]; % Open表，用来存储相邻方向上点，动态更新，每迭代一次，就把最小F值的点删除
Close = []; % Close表，用于存储每迭代一次Open表中的最小F值的点

Finished = 0;
while ~Finished
    [minnode, Open] = FindMinNodeInOpen(Open, F); % 获取Open表中最小F值的节点
    Close = [Close, minnode]; % 并放入到Close表中
    if Map(minnode.x, minnode.y) == SPACE
        Map(minnode.x, minnode.y) = VISITED;
    end
    
    for i = 1:4
        % 遍历上下左右四个方向上的点
        node.x = minnode.x + directs(i, 1);
        node.y = minnode.y + directs(i, 2);
        if node.x > 0 && node.x <= m && node.y > 0 && node.y <= n
            if Map(node.x, node.y) == END
                Close = [Close, node];
                Finished = 1;
                P(node.x, node.y) = minnode;
                break;
            elseif Map(node.x, node.y) ~= SPACE
                continue;
            end
            
            if Map(node.x, node.y) == VISITED % 如果该点被访问过
                if G(node.x, node.y) > G(minnode.x, minnode.y) + 1 % 该点在Close或者Open表中，当比原来路径的G值大时，更新G值
                    G(node.x, node.y) = G(minnode.x, minnode.y) + 1;
                    P(node.x, node.y) = minnode;
                end
            else
                G(node.x, node.y) = G(minnode.x, minnode.y) + 1;
                H(node.x, node.y) = abs(enode.x - node.x) + abs(enode.y - node.y);
                F(node.x, node.y) = G(node.x, node.y) + H(node.x, node.y);
                Open = [Open, node];
                Map(node.x, node.y) = VISITED;
                P(node.x, node.y) = minnode;
            end
        end
    end
end
path = FindPath(P, snode, enode);
for i = 2:length(path)-1
    node = path(i);
    Map(node.x, node.y) = ON_PATH;
end
Map;
end

function [node, Open] = FindMinNodeInOpen(Open, F)
%% 在Open表中查找最小节点
n = length(Open);
if isempty(Open)
    node = [];
    return;
end
node = Open(1);
index = 1;
minF = F(node.x, node.y);
for i = 2:n
    node = Open(i);
    if F(node.x, node.y) < minF
        minF = F(node.x, node.y);
        index = i;
    end
end
node = Open(index);
if index == 1
    if n == 1
        Open = [];
    else
        Open = Open(index+1:n);
    end
elseif index == n
    Open = Open(1:index-1);
else
    Open = [Open(1:index-1), Open(index+1:n)];
end
end

function path = FindPath(P, snode, enode)
%% 根据父节点表、起始点和终点查找路径
path = [enode];
node = enode;
while 1
    node = P(node.x, node.y);
    path = [node path];
    if node.x == snode.x && node.y == snode.y
        return;
    end
end
end

function Astar_Test()
Map =  [
    '22222222222222222222';...
    '20000222220000000002';...
    '20000222220000000002';...
    '20000000000002220002';...
    '22200000222000000002';...
    '20000222222022220002';...
    '20000222220000000002';...
    '20000222220000000002';...
    '20000000000000000002';...
    '22222222222222222222'];
snode.x = 3;
snode.y = 11;
enode.x = 3;
enode.y = 13;
Map(snode.x, snode.y) = 'S';
Map(enode.x, enode.y) = 'E';
Map;
Astar(Map, snode, enode);
end

