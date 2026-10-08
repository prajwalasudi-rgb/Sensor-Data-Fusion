function d = results_dir(sub)
%RESULTS_DIR Absolute path to <repo>/results[/sub]; creates it if missing.
%   Replaces the original hard-coded [pwd '/Results/...'] paths so the
%   scripts work no matter which folder MATLAB is currently in.
root = fileparts(fileparts(fileparts(mfilename('fullpath')))); % src/utils -> repo root
d = fullfile(root, 'results');
if nargin > 0 && ~isempty(sub)
    d = fullfile(d, sub);
end
if ~exist(d, 'dir')
    mkdir(d);
end
end
