classdef nxOccGrid < handle
    properties
        % n-by-9 matrix of obstacles: [xyz]_min, [xyz]_max, r, g, b
        blocks = zeros(0,9);
        margin = 0;
        
        min = zeros(3,1);
        max = zeros(3,1);
        res = zeros(3,1);        
        size = zeros(3,1);
        
        devmin = zeros(3,1);
        devmax = zeros(3,1);

        % origin location
        origin = zeros(3,1);
        origincells = zeros(3,1);
        
        % voxel position in meters
        %pos = {};
        % 3D occupancy grid; 0 if free, 1 if occupied
        map = [];
    end
    
    methods(Static)
        function occgrid = loadMap(filename, res, margin)
            % occgrid = loadMap(filename, res, margin)
            %
            %   Initialize a 3D occupancy grid from a file in the following
            %   format:
            % @filename:
            %   # An example environment
            %   # boundary xmin ymin zmin xmax ymax zmax
            %   # block xmin ymin zmin xmax ymax zmax r g b
            %   # (lower left) (upper right) (color)
            %   boundary 0.0 0.0 0.0 20.0 5.0 6.0
            %   block 3.10 0.0 2.10 3.90 5.0 6.0 255.0 0.0 0.0
            % @res:
            %   3 x 1 vector = map resolution
            % @margin:
            %   1 x 1 scalar = min allowable distance from obstacle
            %
            %
            if( nargin < 3 )
                margin = 0;
            end
            
            fid = fopen(filename);
            lines = textscan(fid, '%s %f %f %f %f %f %f %f %f %f', 'CommentStyle', '#');
            vals = [lines{2:10}];
            types = lines{1};
            %boundary = [];
            blocks = zeros(0,9);
            for k = 1:length(types);
                type = types{k};
                if strcmp(type, 'boundary') == 1
                    boundary = vals(k, 1:end-3);
                elseif strcmp(type, 'block') == 1
                    blocks = [blocks; vals(k, :)];
                else
                    error('Unrecognized type');
                end
            end
            fclose(fid);
            occgrid = nxOccGrid(boundary, blocks, res, margin);
        end
    end
    
    methods
        function occgrid = nxOccGrid(boundary, blocks, res, margin)
            if( nargin < 4 )
                margin = 0;
            end
            boundary = boundary(:);
            MAP = init_map_nx(boundary(1:3),boundary(4:6),res(:),'logical');
            occgrid.min = MAP.min;
            occgrid.max = MAP.max;
            occgrid.size = MAP.size;
            occgrid.res = MAP.res;
            occgrid.devmin = MAP.devmin;
            occgrid.devmax = MAP.devmax;
            occgrid.origin = MAP.origin;
            occgrid.origincells = MAP.origincells;
            
            occgrid.blocks = blocks;
            occgrid.margin = margin;
            occgrid.map = nxOccGrid.updateCSpace_(occgrid.blocks, occgrid.min, occgrid.size, occgrid.res, occgrid.margin);
        end
        
        function updateCSpace(this)
            this.map = updateCSpace_(this.blocks, this.min, ...
                this.size, this.res, this.margin);
        end

        function [b,mapInd,insideMap] = collide(this, xyz)
            % collides: n-by-1 vector; n(k) = 1 if xyz(k, :) is within
            %   the margin of the map according to the discretization
            % xyz: n-by-3 set of points
            
            datac = meters2cells(xyz,this.min.',this.res.'); % n x 3
            [mapInd, insideMap]=sub2ind_map(this.size.',datac);
            
            b = false(size(xyz,1),1);
            b(~insideMap) = true;
            b(insideMap) = this.map(mapInd(insideMap));
        end
        
        function ind = meters2ind(this,xyz)
            % xyz: n-by-3 set of points 
            datac = meters2cells(xyz,this.min.',this.res.'); % n x 3
            ind = subv2ind(this.size.',datac);
        end
            
        function hndl = plot(this,plotDiscrete)
            if nargin < 2
                plotDiscrete = false;
            end
            if ~isempty(this.blocks)
                if plotDiscrete
                    % Plot discretization
                    xyz = cells2meters(ind2subv(this.size.', find(this.map)), this.min.', this.res.'); %n x 3
                    mins = bsxfun(@minus, xyz.', this.res / 2); % 3 x n
                    maxs = bsxfun(@plus, xyz.', this.res / 2); % 3 x n
                    cs = repmat([255; 0; 0], 1, size(mins, 2));
                else
                    % Plot actual blocks
                    mins = bsxfun(@minus, this.blocks(:, 1:3).', this.margin);
                    maxs = bsxfun(@plus, this.blocks(:, 4:6).', this.margin);
                    cs = this.blocks(:, 7:9).';
                end
                hndl = nxOccGrid.block_list_plot(mins, maxs, cs);
            else
                hndl = [];
            end
        end
    end
    
    methods(Static)
        function hndl = block_list_plot(mins, maxs, clr)
            v = [0 0 0; ...
                1 0 0; ...
                1 1 0; ...
                0 1 0; ...
                0 0 1; ...
                1 0 1; ...
                1 1 1; ...
                0 1 1];
            f = [1 2 6 5; ...
                2 3 7 6; ...
                3 4 8 7; ...
                4 1 5 8; ...
                1 2 3 4; ...
                5 6 7 8];
            c = clr / 255;
            
            d   = maxs - mins;
            N   = size(mins,2);
            vl  = zeros(8*N,3);
            fl  = zeros(6*N,4);
            fcl = zeros(6*N,3);
            for k = 1:size(mins,2)
                vs = v;
                vs(:,1) = vs(:,1) * d(1,k);
                vs(:,2) = vs(:,2) * d(2,k);
                vs(:,3) = vs(:,3) * d(3,k);
                vs(:,1) = vs(:,1) + mins(1,k);
                vs(:,2) = vs(:,2) + mins(2,k);
                vs(:,3) = vs(:,3) + mins(3,k);
                vl((k-1)*8+1:k*8,:) = vs;
                fl((k-1)*6+1:k*6,:) = f + (k-1)*8;
                fcl((k-1)*6+1:k*6,:) = [c(:,k)';c(:,k)';c(:,k)';c(:,k)';c(:,k)';c(:,k)'];
            end
            
            bHolding = ishold;
            if(~bHolding)
                hold on;
            end
            hndl = patch('Vertices', vl, 'Faces', fl, 'FaceColor', 'flat', ...
                'FaceVertexCData', fcl, 'FaceAlpha', 0.3);
            if(~bHolding)
                hold off;
            end
        end
    end
    
    methods(Static,Hidden)
        function map = updateCSpace_(blocks, dim_min, sz, res, margin)
            map = false(transpose(sz(:)));
            
            if( ~isempty(blocks) )
                blocks_min = blocks(:,1:3) - sqrt(2)*margin;
                blocks_max = blocks(:,4:6) + sqrt(2)*margin;
                
                blocks_min_cell = meters2cells( blocks_min.',dim_min,res);
                blocks_max_cell = meters2cells( blocks_max.',dim_min,res);
                
                blocks_min_cell(blocks_min_cell < 1) = 1;
                for k = 1:length(sz)
                    toFix = blocks_max_cell(k,:) > sz(k);
                    blocks_max_cell(k,toFix) = sz(k);
                end

                for blk = 1:size(blocks,1)
                    map(blocks_min_cell(1,blk):blocks_max_cell(1,blk),...
                        blocks_min_cell(2,blk):blocks_max_cell(2,blk),...
                        blocks_min_cell(3,blk):blocks_max_cell(3,blk)) = true;
                end
            end
            %blocks_min_ind = subv2ind(sz,blocks_min_cell);
            %blocks_max_ind = subv2ind(sz,blocks_max_cell);
        end
    end
end