all: build/fw.hex build/bootloader.hex build/bootloader_0.hex build/bootloader_1.hex  build/bootloader_2.hex build/bootloader_3.hex build/bootloader_4.hex build/bootloader_5.hex

CC = sdcc -mmcs51 --stack-auto --code-loc 0x400 --code-size 0x1800 --xram-size 0x0200 --xram-loc 0x0000 -Isrc -Iinc
BL_CC = sdcc -mmcs51 --code-loc 0 --code-size 0x400 --xram-size 0 --xram-loc 0x0000 -Isrc -Iinc
AS = sdas8051 -logs

CSRC = $(wildcard src/*.c)
SRC = $(CSRC) $(wildcard src/*.a51)

-include $(CSRC:%=build/%.dep)

BL_CSRC = $(wildcard bootloader_src/*.c)
BL_SRC = $(BL_CSRC) $(wildcard bootloader_src/*.a51)

-include $(BL_CSRC:%=build/%.dep)

build/src/%.c.rel: src/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $< -M | sed -e "s@.*\.rel: @$@: @" > build/$<.dep
	$(CC) -c $< -o $@

build/bootloader_src/%.c.rel: bootloader_src/%.c
	@mkdir -p $(dir $@)
	$(BL_CC) -c $< -M | sed -e "s@.*\.rel: @$@: @" > build/$<.dep
	$(BL_CC) -c $< -o $@

build/%.a51.rel: %.a51
	@mkdir -p $(dir $@)
	cp $< build/$<.a51
	$(AS) build/$<.a51

build/fw.hex: $(SRC:%=build/%.rel)
	@mkdir -p $(dir $@)
	$(CC) $^ -o $@

build/bootloader.hex: $(BL_SRC:%=build/%.rel)
	@mkdir -p $(dir $@)
	$(BL_CC) $^ -o $@

clean:
	rm -fr build

build/bootloader_%.hex: build/bootloader.hex add_id.py
	python add_id.py $< $* $@
