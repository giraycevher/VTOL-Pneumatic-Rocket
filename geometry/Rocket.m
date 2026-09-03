

 % load([dir_name '\Profiles\NoCrank\NoCrank_profiles.mat']);
 

run([base_folder '\Profiles\Rocket\rocket_parameters.m'])
 
load([base_folder '\Profiles\Rocket\Rocket_profiles.mat']);section_max = 3;

 

% % Pp=profil_data.Profile_1;
% % centerp = mean(Pp,1);
% % Pnp=Pp-centerp;
% % Prp = [Pnp(:,1)*cosd(45)+Pnp(:,3)*sind(45),Pnp(:,2),Pnp(:,1)*sind(45)-Pnp(:,3)*cosd(45)] + centerp;
% % profil_data.Profile_1 = Prp;

% profile_dum_cell should be in row cell such that each row contains Nx3
% profiles of each sections

% Scaling
sca_rat = 1;

sections = [1:section_max];   % Select from 1 to 5

if thickness_sw == 1 && elongation_sw == 1 && BOI_gen_sw == 0 
    name_extend = ['_thicker_elongated'];
elseif thickness_sw == 1 && BOI_gen_sw == 0
    name_extend = ['_thicker'];
elseif elongation_sw == 1 && BOI_gen_sw == 0
    name_extend = ['_elongated'];
elseif elongation_sw == 1 && BOI_gen_sw == 1
    name_extend = ['_elongated_for_BOI'];    
else
    name_extend = [];
end
%%

delta_map_x = [ 0 0 0 0 0 0 0 0 0];
delta_map_y = [ 0 0 0 0 0 0 0 0 0];
delta_map_z = [ 0 0 0 0 0 0 0 0 0];
thickness_map_of_vol = [0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3]/1e03;

elongation_of_vol_sta = 2.5/1e03;     % Absoulute quantity, profile 1 of internal volume will be elongated towards -ve x axis direction
elongation_of_vol_end = 5.0/1e03;   % Absoulute quantity, last profile of internal volume will be elongated towards +ve x axis direction




%% Regulation
% Below vectors should be in exactly in the same size with the sections size
delta_map_x = delta_map_x(sections);
delta_map_y = delta_map_y(sections);
delta_map_z = delta_map_z(sections);
thickness_map_of_vol = thickness_map_of_vol(sections);



%% Pointy Nose Settings
nose_length = 0.1; % Nose length measured from the first profile to the apex of the nose
tot_nose_loft_sec_num = 5; % How many surfaces will be implemented till the apex of the nose


