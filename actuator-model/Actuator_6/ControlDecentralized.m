function vdchilda=ControlDecentralized( ex, vi, vd, km )
    
    [~,num] = size(ex);
    
    for itr1=1:1:num;
        excur=ex(:,itr1);        
        vdlcur = dot(excur,vd)*excur;
        vdrcur = vd - vdlcur;
        vdl(:,itr1) = vdlcur;
        vdr(:,itr1) = vdrcur; 
        
        for itr2=1:1:num;
            exother=ex(:,itr2);                    
            
            if itr1 == itr2
                vdcother = 0*exother;
            else
                vdcother = dot(exother,vdr(:,itr1))*exother;
            end
            vdc(:,itr1,itr2) = vdcother;            
        end        
    end
    
    for itr1=1:1:num
        excur=ex(:,itr1); 
        omegakm = 1;
        connectionTerm = excur*0;        
        for itr2=1:1:num            
            if itr1 ~= itr2
                omegakm = omegakm*(1-km(itr2));            
                connectionTerm = connectionTerm + km(itr2)*vdc(:,itr1,itr2);
            end
        end
        
        localTerm = omegakm*vdl(:,itr);
                
        vdchilda(:, itr1) = localTerm  + connectionTerm;    
    end
        
    
end



