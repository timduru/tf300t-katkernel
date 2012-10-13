#include "check.h"
#include "tf300t.h"

int tf300t_partition(struct parsed_partitions *state, int part_num)
{
		Sector sect;
		boot_img_hdr *header;
		unsigned block_size = bdev_logical_block_size(state->bdev);

		header = (struct boot_img_hdr *) read_part_sector(state,
			(SOS_START / block_size), &sect);
		
		if(!header) {
			return -1;
		}
		
		if (memcmp (header->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE) == 0) {
			printk(KERN_INFO "SOS partition - start:%d size:%d\n",
				(SOS_START / block_size), (SOS_SIZE / block_size));
			put_partition(state, (part_num + 1), (SOS_START / block_size),
				(SOS_SIZE / block_size));
		}
		put_dev_sector(sect);
		
		header = (struct boot_img_hdr *) read_part_sector(state,
			(LNX_START / block_size), &sect);
		
		if(!header) {
			return -1;
		}
		
		if (memcmp (header->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE) == 0) {
			printk(KERN_INFO "LNX partition - start:%d size:%d\n",
				(LNX_START / block_size), (LNX_SIZE / block_size));
			put_partition(state, (part_num + 2), (LNX_START / block_size),
				(LNX_SIZE / block_size));
		}
		put_dev_sector(sect);
		
		return 1;
}
