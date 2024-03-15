function align_Ylabels(figureHandle)


%---------------- Input checking
% Only want 1 optional input at most
if nargin > 1
    error('myfuns:align_Ylabels:TooManyInputs', ...
        'requires at most 1 optional input');
end
% Default value for the figure handle is the current figure's handle
if nargin == 0
    figureHandle = gcf;
end

%----------------- Body of the function
% Get the handles of each axis (it will take also the legends' handles)
allAxes = findall(figureHandle,'type','axes');
% Get the handles of each legend
allLegends = findobj(figureHandle,'Type','axes','Tag','legend');

% Index relative to legend handles
legInd = [];

% Remove legend's handles
for k = 1:length(allAxes)
    if ~isempty(find(allAxes(k) == allLegends))
        legInd = [legInd k];
    end
end

allAxes(legInd) = [];

% Number of axes (i.e. subplots)
naxes    = length(allAxes);
% Vector containing the x-position of all the axes in the figure
axxPos   = zeros(naxes,1);

for k = 1:naxes
    axPos  = get(allAxes(k),'Pos');
    axxPos(k) = axPos(1);
end

% Find unique positions (each one represents a column within a figure)
uniquexPos = unique(axxPos);

% Number of columns and rows
ncols = length(uniquexPos);
nrows = naxes/ncols;

% If there's only 1 row, there's no need of the function
if nrows > 1
    for l = 1:ncols

        % Cell containing the yticks of all the subplots in each column
        yticks   = cell(nrows,1);
        % Vector containing the ticks' lengths for each axis
        tmp_max  = zeros(nrows,1);
        % Vector containing the row indexes for each column
        indexes  = 1:nrows;
        % Vector containing the handles of the considered column's axes
        hAxCols = allAxes(axxPos == uniquexPos(l));

        % Scan all the yaxis in the figure and find the longest ticks for each one
        for k = 1:nrows
            yticks{k}   = get(hAxCols(k),'yTickLabel');
            %tmp_max(k)  = size(yticks{k},2);
            tmp_max(k) = size(char(yticks{k}),2);
        end

        % Get the index of ylabel relative to the subplot with longest ticks
        [~, indmax] = max(tmp_max); 

        % Get the handle of the ylabel relative to the subplot with longest
        % ticks
        ylmax = get(hAxCols(indmax),'yLabel');
        % This conversion solves issues with subplot2
        set(ylmax,'Units','pixel')
        % Find its x-position
        ylmaxPos  = get(ylmax,'Pos');
        xpos = ylmaxPos(1);
        % Remove the index of this ylabel from those to be repositioned
        indexes(indmax) = [];

        % Reset the x-position of all the other ticks
        for k = indexes
            yl  = get(hAxCols(k),'yLabel');
            % This conversion solves issues with subplot2
            set(yl,'Units','pixel')
            ylPos = get(yl,'Pos');
            set(yl,'Pos',[xpos ylPos(2) ylPos(3)]);
        end
    end
end
%------------- END OF CODE --------------
%Please send suggestions for improvement of the above template header 
%to Denis Gilbert at this email address: gilbertd@dfo-mpo.gc.ca.
%Your contribution towards improving this template will be acknowledged in
%the "Changes" section of the TEMPLATE_HEADER web page on the Matlab
%Central File Exchange
 