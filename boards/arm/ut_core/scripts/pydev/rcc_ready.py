if request.IsInit:
    pass
else:
    if request.Type == "Read":
        request.Value = 0xFFFFFFFF
    else:
        pass
self.NoisyLog("%s on RCC-stub at 0x%x, value 0x%x" % (str(request.Type), request.Offset, request.Value))
