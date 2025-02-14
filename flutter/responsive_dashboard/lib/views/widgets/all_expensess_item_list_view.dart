import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/all_expensess_item_model.dart';
import 'package:responsive_dashboard/utils/app_images.dart';
import 'package:responsive_dashboard/views/widgets/all_expensess_item.dart';

class AllExpensessItemListView extends StatefulWidget {
  const AllExpensessItemListView({super.key});

  @override
  State<AllExpensessItemListView> createState() =>
      _AllExpensessItemListViewState();
}

class _AllExpensessItemListViewState extends State<AllExpensessItemListView> {
  final items = const [
    AllExpensessItemModel(
        image: Assets.imagesBalance,
        title: 'Balance',
        data: 'Feb 2022',
        price: 20),
    AllExpensessItemModel(
        image: Assets.imagesIncome,
        title: 'Income',
        data: 'jun 2022',
        price: 20),
    AllExpensessItemModel(
        image: Assets.imagesExpenses,
        title: 'Expenses',
        data: 'Mars 2022',
        price: 20),
  ];
  int selectedIndex = 0;
  @override
  Widget build(BuildContext context) {
    return Row(
      children: items.asMap().entries.map((e) {
        int index = e.key;
        var item = e.value;

        if (index == 1) {
          return Expanded(
            child: GestureDetector(
              onTap: () {
                updateIndex(index);
              },
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 12),
                child: AllExpensessItem(
                  itemModel: item,
                  isSelected: selectedIndex == index ? true : false,
                ),
              ),
            ),
          );
        } else {
          return Expanded(
            child: GestureDetector(
              onTap: () {
                updateIndex(index);
              },
              child: AllExpensessItem(
                itemModel: item,
                isSelected: selectedIndex == index ? true : false,
              ),
            ),
          );
        }
      }).toList(),
      // children: items
      //     .map((e) => Expanded(child: AllExpensessItem(itemModel: e)))
      //     .toList(),
    );
  }

  void updateIndex(int index) {
    setState(() {
      selectedIndex = index;
    });
  }
}
