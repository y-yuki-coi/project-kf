function [vdchilda,km]=ComputeVdchilda( ex, vi, vd )
    
    [~,num] = size(ex);
    
    eps1=1e-07;
    eps2=1e-07;
    
    for itr1=1:1:num;
        excur=ex(:,itr1);        
        vdlcur = dot(excur,vd)*excur;
        vdrcur = vd - vdlcur;
        vdl(:,itr1) = vdlcur;
        vdr(:,itr1) = vdrcur; 
        
        %compute km 
        km(itr1)=exp(-4*log(2)*(norm(vdl(:,itr1)-vi(:,itr1))^2+eps1)/(norm(vdl(:,itr1))^2+eps2));
        km(itr1)=0;
        
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
                connectionTerm = connectionTerm + km(itr2)*vdc(:,itr2,itr1);
            end
        end
        
        localTerm = omegakm*vdl(:,itr1);        
        vdchilda(:, itr1) = localTerm  + connectionTerm;    
    end
        
    
end



