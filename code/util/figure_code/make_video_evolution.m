function make_video_evolution()
% For EG. 
% Angjoo Kanazawa <2015-11-25 Wed>
%% Cats
IMG_DIR = '../data/cat_train/';
images = fullfile(IMG_DIR, {'frontal.jpg',
          'jumping.jpg',
          'left_walking.jpg',
          'licking.jpg',
          'rolled.png',
          'running.jpg',
          'sit_curled.png',
          'sitting_profile.jpg',
          'twisting.jpg',
          'walking.jpg',
         });
% Specify image names, call param so we can compute cameras. Recompute cameras & rotate the shape so all shapes are I at camera viewpoint.
base_dirs = {...
    % '../results/CAM_N_FRAME/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_NONE_ulam_10/',
    % '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_BI_uniformC_warm_1.01_to_2_nstep_10_ulam_10/',
    '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontal-jumping-left_walking-licking-rolled-running-sit_curled-sitting_profile-twisting-walking/'
            };
SAVE_DIR = '../movies/cat_eg';
%%% Horses
% IMG_DIR = '../data/horse_train/';
% images = fullfile(IMG_DIR, {...
%     'frontalrun.png',
%     'gallop.png',
%     'graze.jpg',
%     'grazing.png',
%     'leftjog.png',
%     'lookleft.png',
%     'ote.png',
%     'profile.png',
%     'run.png',
%     'straightleft.png',
%     'white.png', ...
%                    });
% % Specify image names, call param so we can compute cameras. Recompute cameras & rotate the shape so all shapes are I at camera viewpoint.
% base_dirs = { ...
%     % '../results/CAM_N_FRAME/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_NONE_ulam_10/',
%     % '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_uniformC_warm_1.01_to_2_nstep_10_ulam_10/',
%     '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontalrun-gallop-graze-grazing-leftjog-lookleft-ote-profile-run-straightleft-white/'
%             };
% SAVE_DIR = '../movies/horse_eg';
%% ----------

% method_names = {'ARAP', 'Uniform', 'Stiffness'};
method_names = {'Stiffness'};

exists_or_mkdir(SAVE_DIR);
meta.articulation = ArticulationEnum.LEARN;

% Just so I can get the 2D-to-3D corresp.
if ~isempty(strfind(SAVE_DIR, 'horse'))
    param = param_horse(images, meta);
else
    param = param_cat(images, meta);
end

blue = [156 199 234]./255;
%% Load each shape, compute camera, and rotate.
for i = 1:param.num_img
    all_shapes = cell(length(base_dirs), 1);
    for j = 1:length(base_dirs)
         shapes = load_shape(fullfile(base_dirs{j}, param.img_name{i}), param.anchor_coords{i}, param.anchors{i});
         all_shapes{j} = shapes;
    end
    
    % Now save
    for j = 1:length(base_dirs)        
        save_name = fullfile(SAVE_DIR, sprintf('%s_%s_evolution', param.img_name{i}, method_names{j}));
        save_video_evolution(all_shapes{j}, save_name);
    end
    
    % Plot them.
    % directions = {[0 1 0], [-1 0 0]};
    % nFrames = 4;
    % steps = 30;
    % for h = 1:2
    %     sfigure(h+1); clf; set(gcf, 'color', 'white');
    %     for f=1:nFrames
    %         for j = 1:length(all_shapes)
    %             sfigure(h+1); subplot(nFrames,length(all_shapes),(f-1)*length(all_shapes)+j); 
    %             hp = patch('Faces',all_shapes{j}.fineTri','Vertices', all_shapes{j}.fineY', 'FaceColor', 'flat', 'FaceVertexCData', blue, 'EdgeColor', 'none');
    %             view(2); axis image off;
    %             axis vis3d;
    %             caxis([0, 1]);
    %             camorbit((f-1)*steps,0,'data',directions{h});
    %             set(hp, 'DiffuseStrength',0.7);
    %             set(hp, 'SpecularColorReflectance',0.5)
    %             set(hp, 'SpecularExponent', 30);
    %             set(hp, 'AmbientStrength', 0.5);
    %             set(hp, 'EdgeLighting', 'gouraud');
    %             camlight('headlight');
    %         end
    %         fprintf('%d/%d: %.1fdeg in [%d %d %d]\n', f, nFrames, f*(steps), directions{h});
    %     end
    % end                   
    % drawnow;
    % keyboard
end



function [all_shapes] = load_shape(res_dir, anchor_coords, anchors)
results = getImageSet(res_dir, 'vtk');
istet = regexpi(results,'.*control*.');
fine = results(cellfun(@isempty,istet));
tet = results(~cellfun(@isempty,istet));
all_shapes = cell(length(fine),1);
DRAW = 0;
for i = 1:length(fine)
    [tetX,tetTri,misc] = read_vtk_more(tet{i});
    tetX = bsxfun(@minus, tetX, mean(tetX, 2));

    [fineY,fineTri, ~] = read_vtk_more(fine{i});
    fineY = bsxfun(@minus, fineY, mean(fineY, 2));
    % Compute the camera.
    [M, t, err, camera ] = get_projection(anchor_coords, fineY(:, anchors));

    % Rotate the shape.
    fineY = camera.R * fineY;
    tetX = camera.R * tetX;

    shape.fineY = fineY;
    shape.fineTri = fineTri;
    shape.tetX = tetX;
    shape.tetTri = tetTri;
    shape.misc = misc;
    all_shapes{i} = shape;

    if DRAW
        sfigure(1); clf;
        blue = [156 199 234]./255; 
        color = repmat(blue, size(fineY,2), 1);
        hp = patch('Faces',fineTri','Vertices', fineY', 'FaceColor', 'interp', 'FaceVertexCData', ...
                   color);
        axis image off;
        sfigure(2); clf;
        hp = patch('Faces',tetTri','Vertices', tetX', 'FaceColor', 'flat', 'FaceVertexCData', ...
                   log(log(misc.distortion)), 'EdgeColor', 'none');
        axis image off;
        lighting gouraud;
        cameratoolbar;
        keyboard
    end
end

function save_video_evolution(shapes, name)
blue = [156 199 234]./255;
DRAW = 0; % Just plays but no save.
if DRAW
    % if isempty(strfind(name, 'Stiffness'))
    %     return;
    % end
    steps = 45;
else    
    steps = 2;
end
nFrames = 360/steps;
angles = [(0:nFrames)*steps];    
% Save surface first.
fname = [name, '.avi'];
name_surf = [name, '_surf'];
exists_or_mkdir(name_surf);

if ~exist(fullfile(name_surf, sprintf('%04d_surf.png', 1))) && exist(fullfile(name, sprintf('%04d_surf.png', 1)))
    system(sprintf('mv %s/*surf.png %s/', name, name_surf));
end

if ~exist(fname, 'file') | ~exist(fullfile(name_surf, sprintf('%04d_surf.png', 1))) 
% if ~exist(name, 'dir')
    hfig = sfigure(100); clf;
    set(hfig, 'OuterPosition', [186   211   1217   776]);
    set(gcf, 'Color', 'white');
    fprintf('working on %s\n', fname);
    all_frames = cell(length(shapes), 1);

    for f=1:length(shapes)
        sfigure(hfig); clf;
        shape = shapes{f};
        hp = patch('Faces',shape.fineTri','Vertices', shape.fineY', 'FaceColor', 'flat', 'FaceVertexCData', blue, 'EdgeColor', 'none');
        view(2); axis image vis3d off;
        camlookat(hp);
        set(hp, 'DiffuseStrength',0.7);
        set(hp, 'SpecularColorReflectance',0.5)
        set(hp, 'SpecularExponent', 30);
        set(hp, 'AmbientStrength', 0.5);
        set(hp, 'EdgeLighting', 'gouraud');
        camlight('headlight');
        frame = getframe(hfig);
        if f == 1
            set(hfig, 'Color', 'k');
            B = getframe(hfig);
            set(hfig, 'Color', 'w');
            A = getframe(hfig);
            alpha = round(sum(double(B.cdata) - double(A.cdata), 3)) / (255 * 3) + 1;
            % keyboard
            frame_size = [size(A.cdata, 1) size(A.cdata, 2)];
        else
            set(hfig, 'Color', 'k');
            B = getframe(hfig, [0   0   frame_size(2)   frame_size(1)]);
            set(hfig, 'Color', 'w');
            A = getframe(hfig, [0   0   frame_size(2)   frame_size(1)]);
            alpha = round(sum(double(B.cdata) - double(A.cdata), 3)) / (255 * 3) + 1;
            % frame = padd_stuff(frame, frame_size);
        end
        all_frames{f} = cat(3, A.cdata, alpha);
        if DRAW
            keyboard
        end
    end
    if ~DRAW
        % Tightly crop the frames & make video.
        A = cat(4, all_frames{:});
        [cropped_frames, vA, vB, bb_rel] = crop_borders(A(:, :, 1:3, :), [255 255 255]', 0);
        cropped_alpha = padarray(A(vA(1):vA(2), vA(3):vA(4), 4, :), [1 1], 'replicate');
        writer = VideoWriter(fname);
        writer.Quality = 100;
        open(writer);
        for i = 1:size(cropped_frames, 4)
            writeVideo(writer, cropped_frames(:, :, :, i));
        end
        close(writer);
        % save as raw PNG.
        exists_or_mkdir(name);
        res = get(0, 'ScreenPixelsPerInch') / 25.4e-3;
        for i = 1:size(cropped_frames, 4)
            im_name = fullfile(name, sprintf('%04d_surf.png', i));
            alpha = cropped_alpha(:, :, i);
            imwrite(cropped_frames(:, :, :, i), im_name, 'Alpha', double(alpha), 'ResolutionUnit', 'meter', 'XResolution', res, 'YResolution', res);
        end
    end
end

%% Draw Tet
fname = [name, '_tet.avi'];
if ~exist(fname, 'file')  | ~exist(fullfile(name, sprintf('%04d.png', 1)))
    hfig = sfigure(101); clf;
    set(hfig, 'OuterPosition', [186   211   1217   776]);
    set(gcf, 'Color', 'white');
    fprintf('working on %s\n', fname);
    all_frames = cell(length(shapes), 1);
    all_stiffness = cell(length(shapes), 1);
    for f=1:length(shapes)
        all_stiffness{f} = shapes{f}.misc.bounds;
    end
    all_stiffness = cat(2, all_stiffness{:});
    all_stiffness(:, 1) =  all_stiffness(:, 1) + 0.01;
    all_stiffness(:, 2:end) =  all_stiffness(:, 2:end) - 1;
    max_val = max(all_stiffness(:));
    min_val = min(all_stiffness(:));
    for f=1:length(shapes)
        sfigure(hfig); clf;
        tetTri = shapes{f}.tetTri';
        tetX = shapes{f}.tetX';
        
        tet_centers = zeros(size(tetTri, 1), 3);
        for i = 1:size(tetTri, 1)
            tet_centers(i, :) = mean(tetX(tetTri(i,:), :));
        end
        scatter3(tet_centers(:, 1), tet_centers(:, 2), tet_centers(:, 3), 100, log(all_stiffness(:, f)), 'filled');
        view(2); axis image vis3d off;
        caxis(log([min_val, max_val]));
        colormap(jet);
        axis image off;
        cameratoolbar;

        % Get frame with black background and white background to compute alpha channel.
        if f == 1
            set(hfig, 'Color', 'k');
            B = getframe(hfig);
            set(hfig, 'Color', 'w');
            A = getframe(hfig);
            alpha = round(sum(double(B.cdata) - double(A.cdata), 3)) / (255 * 3) + 1;
            frame_size = [size(A.cdata, 1) size(A.cdata, 2)];
        else
            set(hfig, 'Color', 'k');
            B = getframe(hfig, [0   0   frame_size(2)   frame_size(1)]);
            set(hfig, 'Color', 'w');
            A = getframe(hfig, [0   0   frame_size(2)   frame_size(1)]);
            alpha = round(sum(double(B.cdata) - double(A.cdata), 3)) / (255 * 3) + 1;
            % frame = padd_stuff(frame, frame_size);
        end
        all_frames{f} = cat(3, A.cdata, alpha);
        if DRAW
            keyboard
        end
    end % End all frames
    
    if ~DRAW
        % Tightly crop the frames & make video.
        A = cat(4, all_frames{:});
        [cropped_frames, vA, vB, bb_rel] = crop_borders(A(:, :, 1:3, :), [255 255 255]', 0);
        cropped_alpha = padarray(A(vA(1):vA(2), vA(3):vA(4), 4, :), [1 1], 'replicate');
        % sfigure; montage(permute(double(cropped_alpha), [1 2 4 3]));
        writer = VideoWriter(fname);
        writer.Quality = 100;
        open(writer);
        for i = 1:size(cropped_frames, 4)
            writeVideo(writer, cropped_frames(:, :, :, i));
        end
        close(writer);
        % save as raw PNG.
        exists_or_mkdir(name);
        res = get(0, 'ScreenPixelsPerInch') / 25.4e-3;
        for i = 1:size(cropped_frames, 4)
            im_name = fullfile(name, sprintf('%04d.png', i));
            % hfig = sfigure(1); clf; imagesc(cropped_frames(:, :, :, i)); axis image off;
            alpha = cropped_alpha(:, :, i);
            imwrite(cropped_frames(:, :, :, i), im_name, 'Alpha', double(alpha), 'ResolutionUnit', 'meter', 'XResolution', res, 'YResolution', res);
        end
    end
    
end

