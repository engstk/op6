CONFIG_TOUCHSCREEN_GOODIX_GT1X=y
CONFIG_GTP_AUTO_UPDATE=y

&qupv3_se12_i2c {
	status = "ok";
	goodix_ts@14{
		compatible = "goodix,gt1x";
		reg = <0x14>;
		interrupt-parent = <&tlmm>;
		interrupts = <125 0x2008>;
		vdd_ana-supply = <&pm8998_l28>;
		goodix,reset-gpio = <&tlmm 99 0x00>;
		goodix,irq-gpio = <&tlmm 125 0x00>;
		goodix,1v8-gpio = <&tlmm 88 0x00>;
		goodix,default-config0 = [
		        03 38 04 E8 08 0A 1D 08 0B E2
		        00 0C 50 3C 39 01 00 00 00 00
		        21 85 85 C6 08 06 03 00 08 0A
		        00 44 00 31 00 00 00 00 00 40
		        3C 1E 17 02 0E 28 88 28 21 60
		        62 C4 09 38 5D 3B A1 53 26 44
		        06 4A 6E C0 54 34 19 18 03 8C
		        50 87 56 83 5C 80 62 7D 68 7C
		        00 00 00 00 00 00 00 57 3C 2D
		        FF 00 0A 0F 1E 08 00 46 00 00
		        00 00 00 64 0A 0B 00 00 01 73
		        0A 32 14 AF 8C 23 00 00 00 00
		        00 00 00 20 00 00 00 00 00 00
		        00 00 00 00 1E 00 DF 07 50 32
		        1F 1E 1B 1D 1C 1A 19 16 17 14
		        15 13 12 11 18 0F 10 0E 0D 0C
		        0A 0B 08 09 07 06 01 00 02 05
		        04 03 1A 18 19 0A 0E 0D 0C 0F
		        0B 03 08 09 07 06 04 05 02 FF
		        FF FF FF FF 00 30 11 14 00 04
		        00 63 81 A2 00 00 00 00 00 00
		        00 00 00 00 00 88 00 00 00 00
		        00 03 14 50 2D 31 14 50 02 44
		        10 00 23 C4 09 26 30 59 01
			];
		goodix,charger-config2 = [
			];
		pinctrl-names = "default";
		pinctrl-0 = <&goodix_int_pull_up>;
	};

	synaptics-rmi-ts@20 {
		compatible = "HWK,synaptics,s3320";
		reg = <0x20>;
		interrupt-parent = <&tlmm>;
		interrupts = <125 0x2008>;
		//vcc_i2c_1v8-supply = <&pm8998_l6>;
		vdd_2v8-supply = <&pm8998_l28>;
		synaptics,tx-rx-num = <15 30>;
		//synaptics,vdd-voltage = <1808000 1808000>;
		synaptics,avdd-voltage = <3008000 3008000>;
		//synaptics,vdd-current = <40000>;
		synaptics,avdd-current = <20000>;
		synaptics,display-coords = <1080 2160>;
		synaptics,panel-coords = <1080 2160>;
		synaptics,reset-gpio = <&tlmm 99 0x00>;
		synaptics,irq-gpio = <&tlmm 125 0x2008>;
		synaptics,1v8-gpio = <&tlmm 88 0x00>;
		oem,support_1080x2160_tp;
		oem,support_hw_poweroff;
		pinctrl-names = "pmx_ts_active", "pmx_ts_suspend";
		pinctrl-0 = <&ts_active>;
		pinctrl-1 = <&ts_int_suspend &ts_reset_suspend>;
	};
};

//overlay for camera end.

&tlmm {
		//add by gu qicai begin
		ts_mux {
			ts_active: ts_active {
				mux {
					pins = "gpio99", "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio99", "gpio125";
					drive-strength = <16>;
					bias-pull-up;
				};
			};

			ts_reset_suspend: ts_reset_suspend {
				mux {
					pins = "gpio99";
					function = "gpio";
				};

				config {
					pins = "gpio99";
					drive-strength = <2>;
					bias-pull-down;
				};
			};

			ts_int_suspend: ts_int_suspend {
				mux {
					pins = "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio125";
					drive-strength = <2>;
					bias-disable;
				};
			};
		};
		//add by gu qicai end

		/*add by gu qicai begin for goodix tp*/
		ts_mux_goodix_tp {
			goodix_int_pull_up: goodix_int_pull_up {
				mux {
					pins = "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio125";
					drive-strength = <16>;
					bias-pull-up;
				};
			};
		};
		/*add by gu qicai end for goodix tp*/
};