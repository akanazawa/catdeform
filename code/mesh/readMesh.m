function [ X, tri ] = readMesh(FName)

[~,~,ext] = fileparts(FName);
switch lower(ext)
    case '.ply'
        [X, tri]=read_ply(FName);
    case '.obj'
        [X, tri]=read_obj(FName);
        X = X';
        tri = tri';
    case '.off'
        [X, tri]=read_off(FName);
        X = X';
        tri = tri';
  case '.mat';
        t = load(FName);
        X = [t.surface.X, t.surface.Y, t.surface.Z];
        tri = t.surface.TRIV;
    otherwise
        error('invalid file type')
end
end

