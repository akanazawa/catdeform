function [coeff] = computeRelativeCoordinates3D(coarseX, coarseTri, fineX, fineTri, prmCoords)

coarseTR = triangulation(coarseTri, coarseX);
fineTR = triangulation(fineTri, fineX);

switch prmCoords.coordsType
    case 'mls',
        % get the weights of all coarse points relative to every fine point
        % (each row is the weights w.r.t. to a fine point)

        % fine basis - the closest fine to every coarse point
        [fineBaseInd, fineBaseDist]  = knnsearch(fineX,coarseX);
        
        % get fine graph (weighted) adjacency matrix
        fprintf('Calculating fine (weighted) adjacency matrix\n');
        fineAdj = triangulation2adjacency(fineTri,fineX);
        
        
        % get coarse graph (weighted) adjacency matrix
        fprintf('Calculating coarse (weighted) adjacency matrix\n');
        coarseAdj = triangulation2adjacency(coarseTri,coarseX);
        
        % calc graph of coarse connected to fine (connect coarse surface to
        % closest fine point)
        coarseBoundaryTri = coarseTR.freeBoundary();
        coarseBoundaryTriInd = unique(coarseBoundaryTri); % get all vertices that participate in boundary
        % connecting matrix coarse_boundary<->fine
        % AJ: N_coarse x N_fine matrix where coarse-boundary and
        % fine vertices (closest to each of those coarse-boundary
        % vertices) have edge with weight = knn distance.
        connectAdj = sparse(coarseBoundaryTriInd, ...
                            fineBaseInd(coarseBoundaryTriInd),...
                            fineBaseDist(coarseBoundaryTriInd),...
                            size(coarseX,1),size(fineX,1));
        % AJ: N_fine + N_coarse by N_fine + N_coarse
        allAdj = [fineAdj, connectAdj'; connectAdj, coarseAdj];
       
        % find the distance between every fine node to every coarse node
        dist = zeros(size(fineX,1), size(coarseX,1));
        % textprogressbar('Approximating distances: ');
        fprintf('Approximating distances: ');
        for i=1:size(coarseX,1),
            curCoarseInd = size(fineX,1)+i;
            cur_dist = perform_dijkstra_fast(allAdj,curCoarseInd);
            dist(:,i) = cur_dist(1:size(fineX,1));
            % textprogressbar(round(i/length(fineBaseInd)*100));
        end
        dist_sqr = dist.^2;
        % textprogressbar(' Done.');
        fprintf(' Done.\n');
        
        % calculate weights
        sigma_sqr = prmCoords.mlsSimga^2;
        W = exp(-dist_sqr/sigma_sqr);
        % get MLS coefficients
        coeff = MLS(coarseX,fineX,W,prmCoords.mlsOrder,1e-5);
        
    otherwise
        error('invalid type')
end
end

