function output_vtk_surf(filename,V,T,v_scalars,t_scalars,v_texture)
ofid = fopen(filename,'w');

fprintf(ofid, '# vtk DataFile Version 3.0\n');
fprintf(ofid,'vtk output\n');
fprintf(ofid,'ASCII\n');
fprintf(ofid,'DATASET POLYDATA\n');
fprintf(ofid,'POINTS %d float\n', size(V,1));
fprintf(ofid,'%g %g %g\n', V');
fprintf(ofid,'POLYGONS %d %d\n', size(T,1), 4*size(T,1));
fprintf(ofid,'3 %d %d %d\n', T'-1);
fprintf(ofid,'\n');

fprintf(ofid,'POINT_DATA %d\n', size(V,1));
if ~isempty(v_scalars)
    f=fields(v_scalars);
    for i=1:length(f)
        cur_f=f{i};
        fprintf(ofid,'SCALARS %s float 1\n',cur_f);
        fprintf(ofid,'LOOKUP_TABLE default\n');
        fprintf(ofid,'%g\n',v_scalars.(cur_f));
    end
end

if exist('v_texture','var') && ~isempty(v_texture)
    f=fields(v_texture);
    for i=1:length(f)
        cur_f=f{i};
        if ~isempty(v_texture.(cur_f)),
            cur_dim=size(v_texture.(cur_f),2);
            fprintf(ofid,'TEXTURE_COORDINATES %s %d float\n',cur_f,cur_dim);
            fprintf(ofid,[repmat('%g ',[1 cur_dim]) '\n'],v_texture.(cur_f)');
        end
    end
end

if ~isempty(t_scalars),
    fprintf(ofid,'CELL_DATA %d\n', size(T,1));
    f=fields(t_scalars);
    for i=1:length(f)
        cur_f=f{i};
        fprintf(ofid,'SCALARS %s float 1\n',cur_f);
        fprintf(ofid,'LOOKUP_TABLE default\n');
        fprintf(ofid,'%g\n',t_scalars.(cur_f));
    end
end

fclose(ofid);
end
