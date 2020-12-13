function res = nearestNeighbour(x,obsi)
    [N,~] = size(obsi);
    dis = [];
    for io = 1:N
        dis = [dis;norm(obsi(io,:)-x(1:2)')];
    end
    res = min(dis);
end