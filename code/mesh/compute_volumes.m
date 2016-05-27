function v = compute_volumes( T,X )
if size(T,2)==3 && size(X,2)==3, % surface
    v = computeSurfAreas(X,T);
else
    v=zeros(size(T,1),1);
    for i=1:size(T,1)
        v(i)=primitive_volume(X(T(i,:),:)',size(X,2));
    end
end


end

