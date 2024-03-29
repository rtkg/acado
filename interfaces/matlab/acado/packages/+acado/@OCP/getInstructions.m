function getInstructions(obj, cppobj, get)
%Used to generate CPP file
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: David Ariens
%    Date: 2010
% 



if (get == 'B')
    
    % HEADER 
    
    if (~isempty(obj.grid))  % GRID
        
        fprintf(cppobj.fileMEX,sprintf('    Grid grid_%s(%s);\n', obj.grid.name, obj.grid.name));
        fprintf(cppobj.fileMEX,sprintf('    OCP %s(grid_%s);\n', obj.name, obj.grid.name));


    elseif(~isempty(obj.tStart)) % NORMAL

        if (~isempty(obj.N))
            fprintf(cppobj.fileMEX,sprintf('    OCP %s(%s, %s, %s);\n', obj.name, obj.tStart.name, obj.tEnd.name, obj.N.name));
        else
            fprintf(cppobj.fileMEX,sprintf('    OCP %s(%s, %s);\n', obj.name, obj.tStart.name, obj.tEnd.name));
        end
    
    else % NO ARGUMENTS
        fprintf(cppobj.fileMEX,sprintf('    OCP %s;\n', obj.name));
        
    end
    
    
    
    % COST FUNCTION    

    for i=1:length(obj.minMayerTerms)   % MIN MAYER
        fprintf(cppobj.fileMEX,sprintf('    %s.minimizeMayerTerm(%s);\n', obj.name, obj.minMayerTerms{i}.toString));
    end
    
    for i=1:length(obj.maxMayerTerms)   % MAX MAYER
        fprintf(cppobj.fileMEX,sprintf('    %s.maximizeMayerTerm(%s);\n', obj.name, obj.maxMayerTerms{i}.toString));
    end
    
    for i=1:length(obj.minLagrangeTerms)% MIN LAGRANGE
        fprintf(cppobj.fileMEX,sprintf('    %s.minimizeLagrangeTerm(%s);\n', obj.name, obj.minLagrangeTerms{i}.toString));
    end
    
    for i=1:length(obj.maxLagrangeTerms)% MAX LAGRANGE
        fprintf(cppobj.fileMEX,sprintf('    %s.maximizeLagrangeTerm(%s);\n', obj.name, obj.maxLagrangeTerms{i}.toString));
    end
    
    for i=1:length(obj.minLSQTermh)     % MIN LSQ
        if (~isempty(obj.minLSQTermS{i}) && ~isempty(obj.minLSQTermr{i})) % S,h,r
            fprintf(cppobj.fileMEX,sprintf('    %s.minimizeLSQ(%s, %s, %s);\n', obj.name, obj.minLSQTermS{i}.name_m, obj.minLSQTermh{i}.name, obj.minLSQTermr{i}.name));
        elseif (~isempty(obj.minLSQTermr{i})) % h,r
            fprintf(cppobj.fileMEX,sprintf('    %s.minimizeLSQ(%s, %s);\n', obj.name, obj.minLSQTermh{i}.name, obj.minLSQTermr{i}.name));
        else % h
            fprintf(cppobj.fileMEX,sprintf('    %s.minimizeLSQ(%s);\n', obj.name, obj.minLSQTermh{i}.name));
        end
    end
    
    for i=1:length(obj.minLSQEndTermh)  % MIN LSQ END TERM
        if (~isempty(obj.minLSQEndTermS{i}) && ~isempty(obj.minLSQEndTermr{i})) % S,h,r
            fprintf(cppobj.fileMEX,sprintf('    %s.minimizeLSQEndTerm(%s, %s, %s);\n', obj.name, obj.minLSQEndTermS{i}.name_m, obj.minLSQEndTermh{i}.name, obj.minLSQEndTermr{i}.name));
        elseif (~isempty(obj.minLSQEndTermr{i})) % h,r
            fprintf(cppobj.fileMEX,sprintf('    %s.minimizeLSQEndTerm(%s, %s);\n', obj.name, obj.minLSQEndTermh{i}.name, obj.minLSQEndTermr{i}.name));
        else % h
            error('error minimizeLSQEndTerm write');
        end
    end
    
    
    
    % SUBJECT TO
    
    for i=1:length(obj.subjectoItems)
        fprintf(cppobj.fileMEX,sprintf('    %s.subjectTo(%s);\n', obj.name, obj.subjectoItems{i}));
    end

    fprintf(cppobj.fileMEX,'\n');
end 
end