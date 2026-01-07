function out = alpha(a,e)
l=length(e);
out=zeros([l,1]);
for i=1:l 
        out(i)=(abs(e(i))).^(a).*sign(e(i));
end
end
