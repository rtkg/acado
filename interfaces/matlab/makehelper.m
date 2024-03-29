function [ ] = makehelper( type, optmake, varargin )
%  This function is a helper file for "make", "makeocp", "makesimulation" and
%  "makeintegrators". See help of these files and use these files directly
%  instead of this one.
% 
%  see also make, makeocp, makesimulation, makeintegrators
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
%    Date: 2009
% 


%% SETTINGS
    if (nargin == 2)
        DEBUG = 0;
        CLEANUP = 0;
        MAKE = 1;
        FORCE = 0;
        
    elseif (nargin == 3 && length(varargin{1}) == 1 && strcmp(varargin{1}{1}, 'clean'))
        DEBUG = 0;
        CLEANUP = 1;
        MAKE = 0;
        FORCE = 0;
        
    elseif (nargin == 3 && length(varargin{1}) == 1 && strcmp(varargin{1}{1}, 'debug'))
        DEBUG = 1;
        CLEANUP = 0;
        MAKE = 1;
        FORCE = 0;
        
    elseif (nargin == 3 && length(varargin{1}) == 1 && strcmp(varargin{1}{1}, 'all'))
        DEBUG = 0;
        CLEANUP = 0;
        MAKE = 1;
        FORCE = 1;
        
    elseif (nargin == 3 && length(varargin{1}) == 2 && strcmp(varargin{1}{1}, 'all') && strcmp(varargin{1}{2}, 'debug'))
        DEBUG = 1;
        CLEANUP = 0;
        MAKE = 1;
        FORCE = 1;
        
    elseif (nargin == 3 && length(varargin{1}) == 2 && strcmp(varargin{1}{1}, 'clean') && strcmp(varargin{1}{2}, 'all'))
        DEBUG = 0;
        CLEANUP = 1;
        MAKE = 1;
        FORCE = 1;
        
    elseif (nargin == 3 && length(varargin{1}) == 3 && strcmp(varargin{1}{1}, 'clean') && strcmp(varargin{1}{2}, 'all') && strcmp(varargin{1}{3}, 'debug'))
        DEBUG = 1;
        CLEANUP = 1;
        MAKE = 1;
        FORCE = 1;
    
    else
        disp( 'Could not understand command. Run "help make".' );
        return;       
    end
    

    addpath(genpath([pwd filesep 'shared']));       % ADD SHARED ASSETS TO PATH
 
        
    % DEBUG WARNINGS
    if ( DEBUG == 1 && ~ispc )
        disp( 'WARNING: Debug mode has been designed for use with gcc only!' );
        disp( 'More information about how to debug: see doc/debugging.txt' );
    elseif (DEBUG == 1 && ispc)
        disp( 'More information about how to debug: see doc/debugging.txt' );
    end

   

    % CONSISTENCY CHECK:
    if ( exist( [pwd, '/make.m'],'file' ) == 0 )
         disp( 'ERROR: Run this make script directly within the directory' );
         disp( '       <ACADOtoolkit-inst-dir>/interfaces/matlab/, please.' );
         return;
    end


    
    % RUN THE AUTOMATIC MODEL DETECTION:
    % (CREATES THE FILE: integrator/model_include.hpp)
    automatic_model_detection_integrator();

    warning off all   
    
    % GET FILES
    [HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX, BINFOLDERMEX] = objects(type);
    BIN_FOLDER = 'bin/';
    IFLAGS    = [ '-I. ', HEADER_PATHS           ];
    
    
    %COMPILER FLAGS
    if (ispc)
        % Microsoft Visual C++ (express) compiler
        CPPFLAGS  = [ IFLAGS, ' -D__cpluplus -D__MATLAB__ -Dsnprintf=_snprintf -Dround=acadoRound -O ' ];    
    else
        % Other compilers
        CPPFLAGS  = [ IFLAGS, ' -D__cpluplus -D__MATLAB__ -O ' ];
    end
    counter = 0 ;
    
    
    % DEBUG FLAGS
    DEBUGFLAGS = '';
    if ( DEBUG == 1 )
       if (ispc)
           DEBUGFLAGS = '-g';
       else
           DEBUGFLAGS = '-g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wfloat-equal -Wshadow';
       end
    end
          
    
    % Extensions
    if (ispc)
        ext = '.obj' ;
    else
        ext = '.o' ;
    end
        
    extmex = ['.' mexext] ;  %mexext is a buildin function
    
    
    
%% CLEANUP
    if (CLEANUP)
        fprintf (1, 'Cleaning up all ACADO files... \n') ;
        delete([BIN_FOLDER 'qpOASES' filesep '*' ext]);
        delete([BIN_FOLDER 'src' filesep '*' ext]);
        delete([BIN_FOLDER 'csparse' filesep '*' ext]);
        delete(['integrator' filesep '*' extmex '*']);
        %delete(['acado' filesep '*' extmex '*']);
        
        fprintf (1, 'Removing ACADO folders from Matlab path... \n') ;
        rmpath(genpath([pwd filesep BIN_FOLDER]));      % REMOVE BIN FOLDER FROM PATH 
        rmpath(genpath([pwd filesep 'shared']));        % REMOVE SHARED ASSETS FROM PATH
        rmpath(genpath([pwd filesep 'integrator']));    % REMOVE INTEGRATORS FROM PATH
        rmpath(genpath([pwd filesep 'acado']));         % REMOVE OCP FROM PATH  
        
        fprintf (1, 'Clean completed. \n') ;
    end
    
    

%% MAKING
    if (MAKE)
        fprintf (1, 'Making ACADO... \n') ;
        
       
        
        % C++ files
        CBINFILES = [];
        for i = 1:length (SRC)
            force_compilation = check_to_compile (SRC{i}, [BIN_FOLDER, BINFOLDER{i} BIN{i}, ext], FORCE) ;
            if (force_compilation)
                cmd = sprintf ('mex -O -c %s -outdir %s %s %s', ...
                    DEBUGFLAGS, [BIN_FOLDER BINFOLDER{i}], CPPFLAGS, SRC{i}) ;
                counter = execute_command (cmd, counter, DEBUG, SRC{i}) ;
            end

            CBINFILES = [CBINFILES ' ' '''' pwd filesep BIN_FOLDER BINFOLDER{i} BIN{i} ext ''''] ; %#ok<AGROW>
        end



        % Mex files
        for i = 1:length (SRCMEX)
            force_compilation = check_to_compile (SRCMEX{i}, [BINFOLDERMEX{i}, BINMEX{i}, extmex], FORCE) ;
            if (force_compilation || counter > 0 || strcmp(BINMEX{i}, 'ACADOintegrators'))  
                cmd = sprintf ('mex -O %s %s %s %s -outdir %s -output %s', ...
                    DEBUGFLAGS, CPPFLAGS, SRCMEX{i}, CBINFILES, BINFOLDERMEX{i}, [BINMEX{i}, extmex]) ;
                counter = execute_command (cmd, counter, DEBUG, SRCMEX{i}) ;
            end
        end

        
        % Compile option files with "makemex"
        if (~isempty(optmake) && ~isempty(optmake.mexfile) && ~isempty(optmake.outputname))
            cmd = sprintf ('mex -O %s %s %s %s -outdir %s -output %s', ...
                DEBUGFLAGS, CPPFLAGS, optmake.mexfile, CBINFILES, optmake.outputdir, [optmake.outputname, extmex]) ;
            counter = execute_command (cmd, counter, DEBUG, optmake.mexfile) ;
        end
 
        % Store important variables in globals.m file
        file = fopen('shared/acadoglobals.m','w');
        fprintf(file,'%% This file is autogenerated while executing the make command.\n');
        fprintf(file,'global ACADO_; ACADO_=[];\n');
        fwrite(file,['ACADO_.pwd = ''',pwd,''';']);
        fprintf(file,'\nACADO_.problemname = '''';');
        fprintf(file,'\nACADO_.modelactive = 0;');
        fprintf(file,sprintf('\nACADO_.mexcall = ''mex -O %s %s %s %s -output %s'';', ...
            DEBUGFLAGS, regexprep(regexprep(CPPFLAGS, '\\', '\\\\'), '''', ''''''), '%%s', regexprep(regexprep(CBINFILES, '\\', '\\\\'), '''', ''''''), ['%%s', extmex]));
        
        fclose(file);
        
        
        
        % PATHS
        addpath(genpath([pwd filesep BIN_FOLDER]));     % ADD BIN FOLDER TO PATH (and subfolders)
        addpath(genpath([pwd filesep 'shared']));       % ADD SHARED ASSETS TO PATH
        
        if (type == 0 || type == 1)
            addpath([pwd filesep 'integrator']);   % ADD INTEGRATORS TO PATH. Only add top folder to path!
        end
        
        if (type == 0 || type == 2 || type == 3)
            addpath([pwd filesep 'acado']);   % ADD acado TO PATH. Only add top folder to path!
            addpath([pwd filesep 'acado' filesep 'functions']);
            addpath( genpath([pwd filesep 'acado' filesep 'packages']) );
        end
        
        fprintf (1, sprintf('\nACADO successfully compiled.\nNeeded to compile %d file(s).\n\n', counter)) ;
        fprintf (1, 'If you need to restart Matlab, run this make file again \n') ;
        fprintf (1, 'to set all paths or run savepath in your console to \n') ;
        fprintf (1, 'save the current search path for future sessions.\n') ;
         
    end
    
    warning on all
end


function counter = execute_command (s, counter, full_logging, shorthand)
    s = strrep (s, '/', filesep) ;
    if (full_logging)
        fprintf (1, '%s  -->  %s\n', shorthand, s);
    else
        if (mod (counter, 20) == 0)
            fprintf (1, '\n') ;
        end
        fprintf (1, '.') ;
    end
    counter = counter + 1 ;
    eval (s) ;
end


function [force_compilation] = check_to_compile (src, bin, force_make)

    d_bin = dir (bin);
    if (force_make || isempty (d_bin))  
        force_compilation = 1;
    else
        d_src = dir (src);
        try
          force_compilation = (datenum(d_bin.date) < datenum(d_src.date)) ;
        catch
          force_compilation = 1;
          disp('Warning: datenum is not working on your system. See http://www.acadotoolkit.org/matlab/faq/datenum.php');
        end
    end
    
end
