import 'package:flutter/material.dart';
import 'package:responsive_dashboard/views/widgets/custom_background_container.dart';
import 'package:responsive_dashboard/views/widgets/my_cards_section.dart';
import 'package:responsive_dashboard/views/widgets/transaction_history.dart';

class MyCardsAndTransactionsHistorySection extends StatelessWidget {
  const MyCardsAndTransactionsHistorySection({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomBackGroundContainer(
      padding: 24,
      child: Column(
        children: [
          MyCardsSection(),
          Divider(
            color: Color(0xffF1F1F1),
            height: 40,
          ),
          TransactionHistory(),
        ],
      ),
    );
  }
}
