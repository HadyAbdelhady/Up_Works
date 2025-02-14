import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/drawer_item_model.dart';
import 'package:responsive_dashboard/utils/app_images.dart';
import 'package:responsive_dashboard/views/widgets/drawer_item.dart';

class DrawerItemListView extends StatefulWidget {
  const DrawerItemListView({
    super.key,
  });

  @override
  State<DrawerItemListView> createState() => _DrawerItemListViewState();
}

class _DrawerItemListViewState extends State<DrawerItemListView> {
  int activeIndex = 0;
  final List<DrawerItemModel> items = const [
    DrawerItemModel(title: 'Dashboard', icon: Assets.imagesDashboard),
    DrawerItemModel(title: 'My Transaction', icon: Assets.imagesMyTransctions),
    DrawerItemModel(title: 'Statistics', icon: Assets.imagesStatistics),
    DrawerItemModel(title: 'Wallet Account', icon: Assets.imagesWalletAccount),
    DrawerItemModel(title: 'My Investments', icon: Assets.imagesMyInvestments),
  ];
  @override
  Widget build(BuildContext context) {
    return SliverList.builder(
      
      itemCount: items.length,
      itemBuilder: (context, index) {
        return GestureDetector(
          onTap: () {
            if (activeIndex != index) {
              setState(() {
                activeIndex = index;
                
              });
            }
          },
          child: Padding(
            padding: const EdgeInsets.only(
              top: 20,
            ),
            child: DrawerItem(
              drawerItemModel: items[index],
              isActive: activeIndex == index,
            ),
          ),
        );
      },
    );
  }
}
