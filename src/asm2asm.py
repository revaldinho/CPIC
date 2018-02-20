import re
import sys

asmline_re = re.compile("\s*?(?P<label>\w*:)?\s*?(?P<ops>.*)")
filename = sys.argv[1]

print ("asm volatile (")
with open (filename,"r") as myfile :
    for l in myfile:
        fields = (l.strip()).split("@")
        mobj = asmline_re.match(fields[0].replace("\t","  "))
        if mobj:
            label = (mobj.groupdict()["label"]).strip() if mobj.groupdict()["label"] else ""
            ops   = (mobj.groupdict()["ops"]).strip() if mobj.groupdict()["ops"] else ""
            comment = "".join(fields[1:])
            if ( label != "" or ops != ""):
                print ( "    \"%-10.10s %-32s\\n\" // %s" % ( label,ops,"".join(fields[1:])))
            elif len(fields) > 1:
                print ("    // %s" % "".join(fields[1:]))
print ("    : // Input Register List")
print ("    : // Output Register List")
print ("    : %s // Clobber List" % (",".join( ["\"r%d\"" %d for d in range(0,11)])))
print (");")
