%% Remove the noise point insdie one class to postprocess images
function [postprocess_label] = postprocess(c_label)
    content=string(c_label);
    % Turn label array to number array
    for i=1:size(c_label,1)
        for j=1:size(c_label,2)
            if content(i,j) == "Road"
                num_array(i,j) = 1;
            elseif content(i,j) == "Car"
                num_array(i,j) = 2;
            elseif content(i,j) == "Pedestrian"
                num_array(i,j) = 3;
            elseif content(i,j) == "Bicyclist"
                num_array(i,j) = 4;
            else
                num_array(i,j) = 5;
            end
        end
    end
    % process number array
    w_size = 4;
    for i=w_size+1:(size(c_label,1)-w_size)
        for j=w_size+1:(size(c_label,2)-w_size)
            array_sort = sort([num_array(i-4),num_array(i-3),num_array(i-2),num_array(i-1), ...
                                   num_array(i),num_array(i+1),num_array(i+2),num_array(i+3),num_array(i+4)]);
            average_filter = (array_sort(5));
            num_array(i,j) = average_filter;
        end
    end
    % Turn number array to label array
    for i=1:size(c_label,1)
        for j=1:size(c_label,2)
            if num_array(i,j) == 1
                label_array(i,j) = "Road";
            elseif num_array(i,j) == 2
                label_array(i,j) = "Car";
            elseif num_array(i,j) == 3
                label_array(i,j) = "Pedestrian";
            elseif num_array(i,j) == 4
                label_array(i,j) = "Bicyclist";
            elseif num_array(i,j) == 5
                label_array(i,j) = "Sky";
            end
        end
    end
    postprocess_label = categorical(label_array);

end