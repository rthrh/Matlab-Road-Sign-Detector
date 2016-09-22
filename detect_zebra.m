function [middle] = detect_zebra(image)

h = size(image, 1); %image height
w = size(image, 2); %image width



check = []; %store row indices where pedestrian crossing-like structure has been detected

counter = 0;
countermax = 0;

for i = 1:h

    % count pixels which are of the same value (0 or 1 since we are working
    % on binarized image) and are connected in one continuous sequence
    % e.g. and input matrix A = [0 0 0 0 1 1 1 0 0 1 1 1]
    % would be processed into output matrix B = [ 4 3 2 3]
    
    
    IMh = image(i,:);
    B = IMh(2:end);
    C = IMh(1:end-1) - B;
    D = find(C~=0);
    if (isempty(D))
       D = 0; 
    end    
    result = [D(1) D(2:end) - D(1:end-1) length(IMh)-D(end)]
    %find median length of prevously gathered sequence's lengths
    resultmed = median(result); 

    %discard values which are too distant from median
    for k = 1:length(result)
        if ((result(k) >= floor(resultmed/2)) && result(k) <= (floor(1.5*resultmed)))
            counter = counter + 1;
            if (counter > countermax)
                countermax = counter;
            end
        else counter = 0;    
        end
    end

    %when enough appropriate pixel sequences have been found, store their
    %incides in [check] matrix
    if (countermax > ceil(length(result)/2))
        check = [check i];  
    end
   
    countermax = 0;
   
end

    %calculate diffirence between [check] row indices. Choose those, which
    %are close to each other (checkdiff < 5)
    checkdiff = diff(check);
    B = checkdiff < 5;
   
   %Matrix B is a logical matrixa. If amount of 'ones' is equal or bigger
   %than half of matrix length, return index of median of stored result. It
   %indicates, that zebra crossing has been marked as detected. Otherwise,
   %return '0'

   if (sum(B) >= floor(0.5*length(B)) && ~isempty(check))
      middle =  median(check);
   else
      middle = 0;
   end

   %discard too big median results - so it will not detect backgound as
   %pedestrian crossing
    if (resultmed > 35)
        middle = 0;
    end;
   
  %check