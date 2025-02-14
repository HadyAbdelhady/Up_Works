import 'package:flutter/material.dart';
import 'package:responsive_dashboard/views/widgets/adaptive_layout_widget.dart';
import 'package:responsive_dashboard/views/widgets/custom_drawer.dart';
import 'package:responsive_dashboard/views/widgets/dashboard_desktop_layout.dart';
import 'package:responsive_dashboard/views/widgets/dashboard_mobile_layout.dart';
import 'package:responsive_dashboard/views/widgets/dashboard_tablet_layout.dart';

class DashBoardView extends StatefulWidget {
  const DashBoardView({super.key});

  @override
  State<DashBoardView> createState() => _DashBoardViewState();
}

class _DashBoardViewState extends State<DashBoardView> {
  final GlobalKey<ScaffoldState> scaffoldkey = GlobalKey();
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: scaffoldkey,
      drawer:
          MediaQuery.sizeOf(context).width < 800 ? const CustomDrawer() : null,
      appBar: MediaQuery.sizeOf(context).width < 800
          ? AppBar(
              elevation: 0,
              backgroundColor: const Color(0xff4db7f2),
              leading: IconButton(
                onPressed: () {
                  scaffoldkey.currentState!.openDrawer();
                },
                icon: const Icon(
                  Icons.menu,
                ),
              ),
            )
          : null,
      backgroundColor: const Color(0xfff7f9fa),
      body: AdaptiveLayoutWidget(
        mobileLayout: (context) => const DashBoardMobileLayout(),
        tabletLayout: (context) => const DashBoardTabletLayout(),
        desktopLayout: (context) => const DashBoardDesktopLayout(),
      ),
    );
  }
}
