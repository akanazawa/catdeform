function vtk2off(vtk_file, out_path)


[X, tri, misc] = read_vtk_more(vtk_file);
write_off(out_path, X', tri');
