name = "test2\ground_truth";

% Load the .mat file
matData = load(name + '.mat');

% Display the variables in the file
disp('Variables in the .mat file:');
disp(fieldnames(matData));

% Extract the variable (assuming it's a matrix or a table)
% Replace 'variable_name' with the actual variable name from the .mat file
data = matData.data;

% Check if the data is in matrix format or needs conversion
if istable(data)
    % Save as CSV using writetable for table data
    writetable(data, name + '.csv');
elseif isnumeric(data) || islogical(data)
    % Save as CSV using writematrix for numeric data
    writematrix(data, name + '.csv');
else
    disp('The variable is not in a format that can be directly saved as a CSV.');
end

disp('Data saved to output.csv');