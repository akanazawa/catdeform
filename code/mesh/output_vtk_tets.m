function output_vtk_tets(filename,V,T,v_scalars,t_scalars)
ofid = fopen(filename,'w');
fprintf(ofid, '# vtk DataFile Version 3.0\n');
fprintf(ofid,'vtk output\n');
fprintf(ofid,'ASCII\n');
fprintf(ofid,'DATASET UNSTRUCTURED_GRID\n');
fprintf(ofid,'POINTS %d float\n', size(V,1));
fprintf(ofid,'%g %g %g\n', V');
fprintf(ofid,'CELLS %d %d\n', size(T,1), 5*size(T,1));
fprintf(ofid,'4 %d %d %d %d\n', T'-1);
fprintf(ofid,'CELL_TYPES %d\n',size(T,1));
fprintf(ofid,'%d\n',ones(size(T,1),1)*10);%tet token
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

fprintf(ofid,'CELL_DATA %d\n', size(T,1));
if ~isempty(t_scalars),
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
