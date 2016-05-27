function make_video()
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
left_facing = [0 0 0 0 0 0 0 0 0 0];
% Specify image names, call param so we can compute cameras. Recompute cameras & rotate the shape so all shapes are I at camera viewpoint.
base_dirs = { '../results/CAM_N_FRAME/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_NONE_ulam_10/',
              '../results/WARM/cat_manual-cat_nomouth_v240f476-tetface_380_maxvol_1/ARAP_BI_uniformC_warm_1.01_to_2_nstep_10_ulam_10/',
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
% left_facing = zeros(length(images), 1);[1 1 1 0 1 0 0 0 0 1 1];
% % Specify image names, call param so we can compute cameras. Recompute cameras & rotate the shape so all shapes are I at camera viewpoint.
% base_dirs = { '../results/CAM_N_FRAME/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_NONE_ulam_10/',
%               '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_uniformC_warm_1.01_to_2_nstep_10_ulam_10/',
%               '../results/WARM/horse_manual-myhorse0_v305f606-tetface_500_maxvol_1/ARAP_BI_slam_warm_0.501187_to_0.0501187_nstep_10_ulam_10_symmetric-stiffness/frontalrun-gallop-graze-grazing-leftjog-lookleft-ote-profile-run-straightleft-white/'
%             };
% SAVE_DIR = '../movies/horse_eg';

%% ----------

method_names = {'ARAP', 'Uniform', 'Stiffness'};

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
         shape = load_shape(fullfile(base_dirs{j}, param.img_name{i}), param.anchor_coords{i}, param.anchors{i});
         all_shapes{j} = shape;
         all_misc(j) = shape.misc;
    end
    % Adjust the distortion color.
    all_dist = log(log([all_misc.distortion]));
    min_dist = min(all_dist(:));
    max_dist = max(all_dist(:));
    mu_dist = mean(all_dist(:));
    std_dist = std(all_dist(:));
    thrs = 2.5*std_dist;
    use_min = max(mu_dist - thrs, min_dist);
    use_max = min(mu_dist + thrs, max_dist);
    
    % Now save
    for j = 1:length(base_dirs)        
        tet_colors = max(0, min((all_dist(:, j) - use_min)./(use_max - use_min), 1));
        all_shapes{j}.tet_colors = tet_colors;
        
        save_name = fullfile(SAVE_DIR, sprintf('%s_%s', param.img_name{i}, method_names{j}));
        save_video(all_shapes{j}, save_name, left_facing(i));
    end
end



function [shape] = load_shape(res_dir, anchor_coords, anchors)
results = getImageSet(res_dir, 'vtk');
istet = regexpi(results,'.*control*.');
fine = results(cellfun(@isempty,istet));
tet = results(~cellfun(@isempty,istet));

[tetX,tetTri,misc] = read_vtk_more(tet{end});
tetX = bsxfun(@minus, tetX, mean(tetX, 2));

[fineY,fineTri, ~] = read_vtk_more(fine{end});
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


function save_video(shape, name, left_facing)

if left_facing
    directions = {[0 -1 0], [-1 0 0]};
else
    directions = {[0 1 0], [-1 0 0]};
end
blue = [156 199 234]./255;
DRAW = 0; % Just plays but no save.
if DRAW
    if isempty(strfind(name, 'Stiffness'))
        return;
    end
    steps = 45;
else    
    steps = 2;
end
nFrames = 360/steps;
angles = [(0:nFrames)*steps];    
check_angles = (0:180/min(steps, 15));
% Save surface first.
fname = [name, '.avi'];
if ~exist(fname, 'file')
% if ~exist(name, 'dir')
    hfig = sfigure(100); clf;
    set(hfig, 'OuterPosition', [186   211   1217   776]);
    set(gcf, 'Color', 'white');
    fprintf('working on %s\n', fname);

    % Figure out the camera settings that works for all.
    all_va = zeros(2*length(check_angles),1); 
    all_pos = zeros(2*length(check_angles),3);
    all_target = zeros(2*length(check_angles),3);
    fprintf('computing camera settings..\n');
    for h = 1:2
        for f=1:length(check_angles)
            sfigure(hfig); clf;
            rotShape = rotationMatrix(directions{h}, -check_angles(f)*pi/180 ) * shape.fineY;
            hp = patch('Faces',shape.fineTri','Vertices', rotShape', 'FaceColor', 'flat', 'FaceVertexCData', blue, 'EdgeColor', 'none');
            axis image vis3d off;
            camtarget([0 0 0]);
            pos = campos();
            campos([0 0 pos(end)]);
            camva('auto')
            all_va((h-1)*length(check_angles) + f) = camva();
            all_pos((h-1)*length(check_angles) + f,:) = campos();        
            all_target((h-1)*length(check_angles) + f,:) = camtarget();        
        end
    end
    all_frames = cell(length(angles) * 2, 1);
    [use_va, max_id] = max(all_va);
    use_pos = all_pos(max_id,:);
    use_target = all_target(max_id,:);
    for h = 1:2
        for f=1:length(angles)
            save_name = fullfile(name, sprintf('frame%03d.png', (h-1)*length(check_angles) + f));
            if exist(save_name, 'file'), continue; end
            sfigure(hfig); clf;
            % Instead of camorbit, orbit the shape.
            rotShape = rotationMatrix(directions{h}, -angles(f)*pi/180 ) * shape.fineY;
            hp = patch('Faces',shape.fineTri','Vertices', rotShape', 'FaceColor', 'flat', 'FaceVertexCData', blue, 'EdgeColor', 'none');
            axis image vis3d off;
            camtarget(use_target);
            campos(use_pos);
            camva(use_va);
            set(hp, 'DiffuseStrength',0.7);
            set(hp, 'SpecularColorReflectance',0.5)
            set(hp, 'SpecularExponent', 30);
            set(hp, 'AmbientStrength', 0.5);
            set(hp, 'EdgeLighting', 'gouraud');
            camlight('headlight');
            % fprintf('%d/%d: %.1fdeg in [%d %d %d]\n', f, length(angles), angles(f), directions{h});
            drawnow
            frame = getframe(hfig);
            if f == 1 & h == 1
                frame = getframe(hfig);
                frame_size = [size(frame.cdata, 1) size(frame.cdata, 2)];
            else
                frame = getframe(hfig, [0   0   frame_size(2)   frame_size(1)]);
            end
            all_frames{(h-1)*length(angles) + f} = frame.cdata;;
            if DRAW
                keyboard
            end
        end
    end
    if ~DRAW
        % Tightly crop the frames & make video.
        A = cat(4, all_frames{:});
        [cropped_frames, vA, vB, bb_rel] = crop_borders(A, [255 255 255]', 0);
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
            alpha = ~all(cropped_frames(:, :, :, i) == 255, 3);
            imwrite(cropped_frames(:, :, :, i), im_name, 'Alpha', double(alpha), 'ResolutionUnit', 'meter', 'XResolution', res, 'YResolution', res);
        end
     end
end

