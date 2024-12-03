function convertMatToCsv(rootDir)
    % Function to recursively convert .mat files to .csv and delete the .mat files

    % Get a list of all files and folders in this folder
    files = dir(rootDir);
    
    % Loop through the list of files
    for i = 1:length(files)
        % Skip '.' and '..' and hidden files/folders
        if files(i).name(1) == '.'
            continue;
        end
        
        % Full path of the current file/folder
        fullPath = fullfile(rootDir, files(i).name);
        
        % If it's a folder, recursively call this function
        if files(i).isdir
            % Check if the folder is "ik" or "imu"
            if strcmp(files(i).name, 'ik') || strcmp(files(i).name, 'imu')
                % Convert the .mat files in these folders
                convertMatFilesInFolder(fullPath);
            else
                % Recursively go deeper into subdirectories
                convertMatToCsv(fullPath);
            end
        end
    end
end

function convertMatFilesInFolder(folderPath)
    % Function to convert all .mat files in a folder to .csv files
    matFiles = dir(fullfile(folderPath, '*.mat'));
    
    % Loop over each .mat file
    for i = 1:length(matFiles)
        matFilePath = fullfile(folderPath, matFiles(i).name);
        
        % Load the .mat file
        data = load(matFilePath);
        
        % Assume the data is stored in a table or structure. Adjust if needed.
        tableName = fieldnames(data);  % Get the name of the variable in .mat
        tableData = data.(tableName{1});  % Access the table
        
        % Convert the file name to .csv
        [~, name, ~] = fileparts(matFiles(i).name);
        csvFilePath = fullfile(folderPath, [name '.csv']);
        
        % Write the table to .csv
        writetable(tableData, csvFilePath);
        
        % Delete the .mat file after conversion
        delete(matFilePath);
    end
end
