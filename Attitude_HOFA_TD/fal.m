function out = fal(a,gama,e)

l=length(e);
out=zeros([l,1]);
for i=1:l 
    if(abs(e(i))>gama)
        out(i)=(abs(e(i))).^(a).*sign(e(i));
    else
        out(i)=e(i)/gama^(1-a);
    end
end
out=e;
end

