function sim_out = run_gfl_cosim(build_level, stop_time)
%RUN_GFL_COSIM Launch the GFL controller executable and matching plant model.
arguments
    build_level (1,1) double {mustBeMember(build_level, 1:5)}
    stop_time (1,1) double {mustBePositive} = NaN
end

root = fileparts(mfilename('fullpath'));
model = model_for_level(build_level);
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_simulink.exe');
settings_header = fullfile(root, 'sdpe_mgr', 'sdpe_pgs_inv_gfl_simulate_settings.h');

if ~isfile(exe)
    error('GFL:SILExecutableMissing', 'Build Debug|x64 first: %s', exe);
end
assert_level(settings_header, build_level);
assert_executable_current(exe, settings_header);

model_file = fullfile(root, [model '_2022b.slx']);
if ~isfile(model_file)
    error('GFL:ModelMissing', 'Expected the R2022b-compatible model: %s', model_file);
end
model_handle = load_system(model_file);
model_name = get_param(model_handle, 'Name');
model_cleanup = onCleanup(@() close_system(model_name, 0)); %#ok<NASGU>

controller = start_controller(exe, root);
controller_cleanup = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(0.25);

if isnan(stop_time)
    sim_out = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
else
    sim_out = sim(model_name, 'StopTime', num2str(stop_time, 17), ...
        'ReturnWorkspaceOutputs', 'on');
end
end

function model = model_for_level(level)
if level <= 2 || level == 5
    model = 'DP_STD_MDL_DCAC_3ph_2level_resload';
else
    model = 'DP_STD_MDL_DCAC_3ph_2level_gridconn';
end
end

function assert_level(settings_header, level)
header = fileread(settings_header);
token = regexp(header, '#define\s+BUILD_LEVEL\s+\((\d)\)', 'tokens', 'once');
if isempty(token) || str2double(token{1}) ~= level
    error('GFL:BuildLevelMismatch', ...
        'Generated settings do not select BUILD_LEVEL=%d.', level);
end
end

function assert_executable_current(exe, settings_header)
exe_info = dir(exe);
header_info = dir(settings_header);
if exe_info.datenum < header_info.datenum
    error('GFL:SILExecutableStale', ...
        ['The controller executable is older than the generated SDPE settings. ' ...
         'Rebuild Debug|x64 before running co-simulation: %s'], exe);
end
end

function process = start_controller(exe, root)
info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe;
info.WorkingDirectory = root;
info.UseShellExecute = false;
info.CreateNoWindow = true;
process = System.Diagnostics.Process.Start(info);
end

function stop_controller(process)
try
    if ~isempty(process) && ~process.HasExited
        process.Kill;
        process.WaitForExit(2000);
    end
catch
end
end
