function path = Astar(Map, snode, enode)
%%
% A*�㷨��Ҫ�������������������·������������ڵĵ����·��Ȩֵ�������㲻����
% A���洢���ڵ�֮��ľ������
% snode����ʼ�ڵ�
% enode�������ڵ�

% ���ò���
if nargin == 0
    Astar_Test();
    return;
end

%���MixedPathplanning
Map(snode.x, snode.y) = 'S';
Map(enode.x, enode.y) = 'E';


[m, n] = size(Map);
H = Inf*ones(m, n); % �洢��ǰ�㵽Ŀ����·�����۹���
G = zeros(m, n); % �洢��ʼ�㵽��ǰ���·������ʵ��ֵ
F = Inf*ones(m, n); % ��ʼ�㾭����ǰ�㵽Ŀ���Ĵ���
pnode.x = 0;
pnode.y = 0;
P = repmat(pnode,[m, n]); % �õ�ĸ��ڵ�

VISITED = '-'; % �����ʹ�
WALL = '2'; %         W
START = 'S'; % ���
SPACE = '0'; % �յ�    .
ON_PATH = '*'; % �ڽ��·����
END = 'E'; % �յ�

directs = [[1, 0]; [-1, 0]; [0, 1]; [0, -1]];

%%
% ������ʼ��
H(snode.x, snode.y) = abs(enode.x - snode.x) + abs(enode.y - snode.y);
F(snode.x, snode.y) = G(snode.x, snode.y) + H(snode.x, snode.y);

Open = [snode]; % Open�������洢���ڷ����ϵ㣬��̬���£�ÿ����һ�Σ��Ͱ���СFֵ�ĵ�ɾ��
Close = []; % Close�����ڴ洢ÿ����һ��Open���е���СFֵ�ĵ�

Finished = 0;
while ~Finished
    [minnode, Open] = FindMinNodeInOpen(Open, F); % ��ȡOpen������СFֵ�Ľڵ�
    Close = [Close, minnode]; % �����뵽Close����
    if Map(minnode.x, minnode.y) == SPACE
        Map(minnode.x, minnode.y) = VISITED;
    end
    
    for i = 1:4
        % �������������ĸ������ϵĵ�
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
            
            if Map(node.x, node.y) == VISITED % ����õ㱻���ʹ�
                if G(node.x, node.y) > G(minnode.x, minnode.y) + 1 % �õ���Close����Open���У�����ԭ��·����Gֵ��ʱ������Gֵ
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
%% ��Open���в�����С�ڵ�
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
%% ���ݸ��ڵ����ʼ����յ����·��
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

