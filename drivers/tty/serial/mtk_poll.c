// SPDX-License-Identifier: GPL-2.0

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define MTK_UART_RBR	0x00
#define MTK_UART_THR	0x00
#define MTK_UART_IER	0x04
#define MTK_UART_FCR	0x08
#define MTK_UART_LCR	0x0c
#define MTK_UART_MCR	0x10
#define MTK_UART_LSR	0x14

#define MTK_UART_LSR_DR		0x01
#define MTK_UART_LSR_OE		0x02
#define MTK_UART_LSR_PE		0x04
#define MTK_UART_LSR_FE		0x08
#define MTK_UART_LSR_BI		0x10
#define MTK_UART_LSR_THRE	0x20
#define MTK_UART_LSR_TEMT	0x40

#define MTK_UART_FCR_ENABLE	0x01
#define MTK_UART_FCR_CLEAR_RCVR	0x02
#define MTK_UART_FCR_CLEAR_XMIT	0x04

#define MTK_UART_LCR_WLEN8	0x03
#define MTK_UART_LCR_BREAK	0x40

#define MTK_UART_MCR_DTR	0x01
#define MTK_UART_MCR_RTS	0x02

#define MTK_POLL_IDLE_NS	2000000ULL
#define MTK_POLL_BURST_NS	100000ULL
#define MTK_POLL_MAX_CHARS	64

struct mtk_poll_port {
	struct uart_port port;
	struct hrtimer poll;
};

static inline struct mtk_poll_port *to_mtk_poll(struct uart_port *port)
{
	return container_of(port, struct mtk_poll_port, port);
}

static inline u32 mtk_read(struct uart_port *port, unsigned int reg)
{
	return readl(port->membase + reg);
}

static inline void mtk_write(struct uart_port *port, unsigned int reg, u32 v)
{
	writel(v, port->membase + reg);
}

static enum hrtimer_restart mtk_poll_rx(struct hrtimer *t)
{
	struct mtk_poll_port *mp = container_of(t, struct mtk_poll_port, poll);
	struct uart_port *port = &mp->port;
	unsigned long flags;
	unsigned int lsr, ch, flag, max = MTK_POLL_MAX_CHARS;
	bool got = false;
	ktime_t next;

	uart_port_lock_irqsave(port, &flags);
	while (max-- && ((lsr = mtk_read(port, MTK_UART_LSR)) & MTK_UART_LSR_DR)) {
		ch = mtk_read(port, MTK_UART_RBR) & 0xff;
		flag = TTY_NORMAL;
		port->icount.rx++;
		if (lsr & MTK_UART_LSR_BI) {
			port->icount.brk++;
			flag = TTY_BREAK;
		} else if (lsr & MTK_UART_LSR_PE) {
			port->icount.parity++;
			flag = TTY_PARITY;
		} else if (lsr & MTK_UART_LSR_FE) {
			port->icount.frame++;
			flag = TTY_FRAME;
		} else if (lsr & MTK_UART_LSR_OE) {
			port->icount.overrun++;
		}
		uart_insert_char(port, lsr & MTK_UART_LSR_OE,
				 MTK_UART_LSR_OE, ch, flag);
		got = true;
	}
	tty_flip_buffer_push(&port->state->port);
	uart_port_unlock_irqrestore(port, flags);

	next = ktime_set(0, got ? MTK_POLL_BURST_NS : MTK_POLL_IDLE_NS);
	hrtimer_forward(t, ktime_get(), next);
	return HRTIMER_RESTART;
}

static void mtk_tx_drain(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	unsigned char c;

	if (port->x_char) {
		while (!(mtk_read(port, MTK_UART_LSR) & MTK_UART_LSR_THRE))
			cpu_relax();
		mtk_write(port, MTK_UART_THR, port->x_char);
		port->x_char = 0;
		return;
	}
	if (uart_tx_stopped(port))
		return;
	while (!kfifo_is_empty(&tport->xmit_fifo)) {
		if (!(mtk_read(port, MTK_UART_LSR) & MTK_UART_LSR_THRE))
			break;
		if (!uart_fifo_get(port, &c))
			break;
		mtk_write(port, MTK_UART_THR, c);
	}
	if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static unsigned int mtk_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int lsr;

	uart_port_lock_irqsave(port, &flags);
	lsr = mtk_read(port, MTK_UART_LSR);
	uart_port_unlock_irqrestore(port, flags);
	if ((lsr & (MTK_UART_LSR_THRE | MTK_UART_LSR_TEMT)) ==
	    (MTK_UART_LSR_THRE | MTK_UART_LSR_TEMT))
		return TIOCSER_TEMT;
	return 0;
}

static void mtk_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int mtk_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void mtk_stop_tx(struct uart_port *port)
{
}

static void mtk_start_tx(struct uart_port *port)
{
	mtk_tx_drain(port);
}

static void mtk_stop_rx(struct uart_port *port)
{
}

static void mtk_enable_ms(struct uart_port *port)
{
}

static void mtk_break_ctl(struct uart_port *port, int ctl)
{
	unsigned long flags;
	u32 lcr;

	uart_port_lock_irqsave(port, &flags);
	lcr = mtk_read(port, MTK_UART_LCR);
	if (ctl)
		lcr |= MTK_UART_LCR_BREAK;
	else
		lcr &= ~MTK_UART_LCR_BREAK;
	mtk_write(port, MTK_UART_LCR, lcr);
	uart_port_unlock_irqrestore(port, flags);
}

static int mtk_startup(struct uart_port *port)
{
	struct mtk_poll_port *mp = to_mtk_poll(port);
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	mtk_write(port, MTK_UART_IER, 0);
	mtk_write(port, MTK_UART_FCR,
		  MTK_UART_FCR_ENABLE | MTK_UART_FCR_CLEAR_RCVR |
		  MTK_UART_FCR_CLEAR_XMIT);
	mtk_write(port, MTK_UART_LCR, MTK_UART_LCR_WLEN8);
	mtk_write(port, MTK_UART_MCR, MTK_UART_MCR_DTR | MTK_UART_MCR_RTS);
	uart_port_unlock_irqrestore(port, flags);

	hrtimer_start(&mp->poll, ktime_set(0, MTK_POLL_IDLE_NS),
		      HRTIMER_MODE_REL);
	return 0;
}

static void mtk_shutdown(struct uart_port *port)
{
	struct mtk_poll_port *mp = to_mtk_poll(port);

	hrtimer_cancel(&mp->poll);
	mtk_write(port, MTK_UART_IER, 0);
}

static void mtk_set_termios(struct uart_port *port, struct ktermios *termios,
			    const struct ktermios *old)
{
	unsigned long flags;
	u32 lcr = 0;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr |= 0x00;
		break;
	case CS6:
		lcr |= 0x01;
		break;
	case CS7:
		lcr |= 0x02;
		break;
	case CS8:
	default:
		lcr |= MTK_UART_LCR_WLEN8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr |= 0x04;
	if (termios->c_cflag & PARENB)
		lcr |= 0x08;
	if (!(termios->c_cflag & PARODD))
		lcr |= 0x10;

	uart_port_lock_irqsave(port, &flags);
	mtk_write(port, MTK_UART_LCR, lcr);
	port->read_status_mask = MTK_UART_LSR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= MTK_UART_LSR_FE | MTK_UART_LSR_PE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= MTK_UART_LSR_BI;
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= MTK_UART_LSR_PE | MTK_UART_LSR_FE;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= MTK_UART_LSR_BI;
	uart_port_unlock_irqrestore(port, flags);
}

static const char *mtk_type(struct uart_port *port)
{
	return "sudomkabacon";
}

static struct mtk_poll_port *mtk_poll_port0;

static void mtk_console_putchar(struct uart_port *port, unsigned char c)
{
	while (!(mtk_read(port, MTK_UART_LSR) & MTK_UART_LSR_THRE))
		cpu_relax();
	mtk_write(port, MTK_UART_THR, c);
}

static void mtk_console_write(struct console *co, const char *s,
			      unsigned int count)
{
	struct uart_port *port;
	unsigned long flags;
	int locked;

	if (!mtk_poll_port0)
		return;
	port = &mtk_poll_port0->port;
	if (co->index < 0 || co->index != port->line)
		return;

	if (oops_in_progress)
		locked = uart_port_trylock_irqsave(port, &flags);
	else {
		uart_port_lock_irqsave(port, &flags);
		locked = 1;
	}

	while (count--) {
		if (*s == '\n')
			mtk_console_putchar(port, '\r');
		mtk_console_putchar(port, *s++);
	}

	if (locked)
		uart_port_unlock_irqrestore(port, flags);
}

static int mtk_console_setup(struct console *co, char *options)
{
	if (co->index < 0)
		co->index = 0;
	if (!mtk_poll_port0 || !mtk_poll_port0->port.membase)
		return -ENXIO;

	return 0;
}

static struct uart_driver mtk_poll_reg;

static struct console mtk_poll_console = {
	.name = "ttyGCE",
	.write = mtk_console_write,
	.device = uart_console_device,
	.setup = mtk_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &mtk_poll_reg,
};

static void mtk_config_port(struct uart_port *port, int flags)
{
	port->type = PORT_16550A;
}

static int mtk_request_port(struct uart_port *port)
{
	return 0;
}

static void mtk_release_port(struct uart_port *port)
{
}

static const struct uart_ops mtk_poll_ops = {
	.tx_empty	= mtk_tx_empty,
	.set_mctrl	= mtk_set_mctrl,
	.get_mctrl	= mtk_get_mctrl,
	.stop_tx	= mtk_stop_tx,
	.start_tx	= mtk_start_tx,
	.stop_rx	= mtk_stop_rx,
	.enable_ms	= mtk_enable_ms,
	.break_ctl	= mtk_break_ctl,
	.startup	= mtk_startup,
	.shutdown	= mtk_shutdown,
	.set_termios	= mtk_set_termios,
	.type		= mtk_type,
	.request_port	= mtk_request_port,
	.release_port	= mtk_release_port,
	.config_port	= mtk_config_port,
};

static struct uart_driver mtk_poll_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "mtk_poll",
	.dev_name	= "ttyGCE",
	.major		= 0,
	.minor		= 0,
	.nr		= 1,
	.cons		= &mtk_poll_console,
};

static int mtk_poll_probe(struct platform_device *pdev)
{
	struct mtk_poll_port *mp;
	struct resource *res;
	int id, ret;

	mp = devm_kzalloc(&pdev->dev, sizeof(*mp), GFP_KERNEL);
	if (!mp)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	mp->port.mapbase = res->start;
	mp->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mp->port.membase))
		return PTR_ERR(mp->port.membase);

	id = of_alias_get_id(pdev->dev.of_node, "serial");
	if (id < 0)
		id = 0;

	mp->port.dev = &pdev->dev;
	mp->port.iotype = UPIO_MEM32;
	mp->port.regshift = 0;
	mp->port.fifosize = 16;
	mp->port.ops = &mtk_poll_ops;
	mp->port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	mp->port.line = id;
	mp->port.type = PORT_UNKNOWN;
	mp->port.irq = 0;
	hrtimer_setup(&mp->poll, mtk_poll_rx, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);

	platform_set_drvdata(pdev, mp);
	if (!mtk_poll_port0)
		mtk_poll_port0 = mp;
	ret = uart_add_one_port(&mtk_poll_reg, &mp->port);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "poll-mode UART at %pap (%s%d)\n",
		 &res->start, mtk_poll_reg.dev_name, id);
	return 0;
}

static void mtk_poll_remove(struct platform_device *pdev)
{
	struct mtk_poll_port *mp = platform_get_drvdata(pdev);

	uart_remove_one_port(&mtk_poll_reg, &mp->port);
}
static const struct of_device_id mtk_poll_dt[] = {
	{ .compatible = "mediatek,mt6577-uart-poll" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_poll_dt);

static struct platform_driver mtk_poll_driver = {
	.probe	= mtk_poll_probe,
	.remove	= mtk_poll_remove,
	.driver	= {
		.name = "mtk-uart-poll",
		.of_match_table = mtk_poll_dt,
	},
};

static int __init mtk_poll_init(void)
{
	int ret = uart_register_driver(&mtk_poll_reg);

	if (ret)
		return ret;
	ret = platform_driver_register(&mtk_poll_driver);
	if (ret)
		uart_unregister_driver(&mtk_poll_reg);
	return ret;
}

static void __exit mtk_poll_exit(void)
{
	platform_driver_unregister(&mtk_poll_driver);
	uart_unregister_driver(&mtk_poll_reg);
}

module_init(mtk_poll_init);
module_exit(mtk_poll_exit);

MODULE_DESCRIPTION("MediaTek IRQ-less driver");
MODULE_LICENSE("GPL");
