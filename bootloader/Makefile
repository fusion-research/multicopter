all: build/fw.hex

CC = sdcc -mmcs51 --code-size 0x400 --xram-size 0 --xram-loc 0x0000 -Isrc -Iinc
AS = sdas8051 -logs

CSRC = $(wildcard src/*.c)
SRC = $(CSRC) $(wildcard src/*.a51)

-include $(CSRC:%=build/%.dep)

build/%.c.rel: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $< -M | sed -e "s@.*\.rel: @$@: @" > build/$<.dep
	$(CC) -c $< -o $@

build/%.a51.rel: %.a51
	@mkdir -p $(dir $@)
	cp $< build/$<.a51
	$(AS) build/$<.a51

build/fw.hex: $(SRC:%=build/%.rel)
	@mkdir -p $(dir $@)
	$(CC) $^ -o $@

clean:
	rm -fr build
