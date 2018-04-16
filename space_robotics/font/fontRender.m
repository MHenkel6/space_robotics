classdef fontRender < handle
    properties
        font;
    end
    
    methods
        function self = fontRender()
            self.font = createFont;
        end
        
        %% create 4x3xN moves matrix from font, for specified character
        % 1st row: [xyz] start position
        % 2nd row: [xyz] end position
        % 3rd row: [] rotation direction (+1/-1, 0 if straight line)
        % 4th row: [xyz] rotatation center point
        function movements = sequentializeCharacter(self, character, plane, origin, size)
            sequence = self.font(character(1));
            movements = zeros(4, 3, length(sequence));
            for ii = 1:length(sequence)
                s = sequence{ii};
                movements(1, :, ii) = (size*(plane * s{1}') + origin)';
                movements(2, :, ii) = (size*(plane * s{2}') + origin)';
                if s{3}(3) == 0
                    movements(3, :, ii) = [0,0,0];
                else
                    movements(3, :, ii) = [0,0,0]+s{3}(3);
                end
                movements(4, :, ii) = (size*(plane * s{3}(1:2)') + origin)';
            end
        end
        
        %% create 4x3xN moves matrix for a character array/string
        function movements = sequentializeText(self, stringIn)
            stringMatch = regexp(stringIn, '[a-z ]*', 'ignorecase', 'match');
            stringIn = upper(strcat(stringMatch{:}));
            plane = [[1;0;0], [0;1;0]];
            origin = [0;0;0];
            size = 1.0;
            movements = [];
            lastEndPoint = origin';
            for character = stringIn
                if character == ' '
                    origin = origin + 1.1*plane(:,1)*size;
                    originMove = [lastEndPoint ; origin'; 0,0,0; 0,0,0];
                    if ~all(originMove(:) == 0)
                        movements = cat(3, movements, originMove);
                    end
                else
                    nextLetter = self.sequentializeCharacter(character, plane, origin, size);
                    originMove = [lastEndPoint ; nextLetter(1,:,1); 0,0,0; 0,0,0];
                    if ~all(originMove(:) == 0)
                        movements = cat(3, movements, originMove, nextLetter);
                    else
                        movements = cat(3, movements, nextLetter);
                    end
                end
                origin = origin + 1.1*plane(:,1)*size;
                lastEndPoint = movements(2,:,end);
            end
            
        end
    end
end